/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import com.google.gson.*;
import edu.wpi.cscore.CameraServerJNI;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;
import org.opencv.core.RotatedRect;

import java.io.IOException;
import java.lang.annotation.Target;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ],
               "stream": {                              // optional
                   "properties": [
                       {
                           "name": <stream property name>
                           "value": <stream property value>
                       }
                   ]
               }
           }
       ]
   }
 */

public final class Main {
	private static String configFile = "/boot/frc.json";

	@SuppressWarnings("MemberName")
	public static class CameraConfig {
		public String name;
		public String path;
		public JsonObject config;
		public JsonElement streamConfig;
	}

	public static int team;
	public static boolean server;
	public static List<CameraConfig> cameraConfigs = new ArrayList<>();

	private Main() {
	}

	/**
	 * Report parse error.
	 */
	public static void parseError(String str) {
		System.err.println("config error in '" + configFile + "': " + str);
	}

	/**
	 * Read single camera configuration.
	 */
	public static boolean readCameraConfig(JsonObject config) {
		CameraConfig cam = new CameraConfig();

		// name
		JsonElement nameElement = config.get("name");
		if (nameElement == null) {
			parseError("could not read camera name");
			return false;
		}
		cam.name = nameElement.getAsString();

		// path
		JsonElement pathElement = config.get("path");
		if (pathElement == null) {
			parseError("camera '" + cam.name + "': could not read path");
			return false;
		}
		cam.path = pathElement.getAsString();

		// stream properties
		cam.streamConfig = config.get("stream");

		cam.config = config;

		cameraConfigs.add(cam);
		return true;
	}

	/**
	 * Read configuration file.
	 */
	@SuppressWarnings("PMD.CyclomaticComplexity")
	public static boolean readConfig() {
		// parse file
		JsonElement top;
		try {
			top = new JsonParser().parse(Files.newBufferedReader(Paths.get(configFile)));
		} catch (IOException ex) {
			System.err.println("could not open '" + configFile + "': " + ex);
			return false;
		}

		// top level must be an object
		if (!top.isJsonObject()) {
			parseError("must be JSON object");
			return false;
		}
		JsonObject obj = top.getAsJsonObject();

		// team number
		JsonElement teamElement = obj.get("team");
		if (teamElement == null) {
			parseError("could not read team number");
			return false;
		}
		team = teamElement.getAsInt();

		// ntmode (optional)
		if (obj.has("ntmode")) {
			String str = obj.get("ntmode").getAsString();
			if ("client".equalsIgnoreCase(str)) {
				server = false;
			} else if ("server".equalsIgnoreCase(str)) {
				server = true;
			} else {
				parseError("could not understand ntmode value '" + str + "'");
			}
		}

		// cameras
		JsonElement camerasElement = obj.get("cameras");
		if (camerasElement == null) {
			parseError("could not read cameras");
			return false;
		}
		JsonArray cameras = camerasElement.getAsJsonArray();
		for (JsonElement camera : cameras) {
			if (!readCameraConfig(camera.getAsJsonObject())) {
				return false;
			}
		}

		return true;
	}

	/**
	 * Start running the camera.
	 */
	public static VideoSource startCamera(CameraConfig config) {
		System.out.println("Starting camera '" + config.name + "' on " + config.path);
		CameraServer inst = CameraServer.getInstance();
		UsbCamera camera = new UsbCamera(config.name, config.path);
		inst.addCamera(camera); // For some reason startAutomaticCapture doesn't return MjpegServer in this version
		MjpegServer server = inst.addServer("serve_" + camera.getName());
		server.setSource(camera);

		Gson gson = new GsonBuilder().create();

		camera.setConfigJson(gson.toJson(config.config));
		camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

		return camera;
	}

	/**
	 * Main.
	 */
	public static void main(String... args) {
		if (args.length > 0) {
			configFile = args[0];
		}

		// read configuration
		if (!readConfig()) {
			return;
		}

		// start NetworkTables
		NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
		if (server) {
			System.out.println("Setting up NetworkTables server");
			ntinst.startServer();
		} else {
			System.out.println("Setting up NetworkTables client for team " + team);
			ntinst.startClientTeam(team);
		}

		// start cameras
		List<VideoSource> cameras = new ArrayList<>();
		for (CameraConfig cameraConfig : cameraConfigs) {
			cameras.add(startCamera(cameraConfig));
		}

		NetworkTable visionRoot = ntinst.getTable("vision");
		NetworkTable contours = visionRoot.getSubTable("contours");
		NetworkTable targetPairs = visionRoot.getSubTable("targetPairs");

		// start image processing on camera 0 if present
		if (cameras.size() >= 1) {
		    System.out.println("Its working");
			VisionThread visionThread = new VisionThread(cameras.get(0),
							new VisionTargetPipeline(cameras.get(0)), pipeline -> {
				double[] x = new double[pipeline.contourBB.size()];
				double[] y = new double[pipeline.contourBB.size()];
				double[] angle = new double[pipeline.contourBB.size()];
				double[] w = new double[pipeline.contourBB.size()];
				double[] h = new double[pipeline.contourBB.size()];
				int i = 0;
				for (RotatedRect target : pipeline.contourBB) {
					x[i] = target.center.x;
					y[i] = target.center.y;
					angle[i] = target.angle;
					w[i] = target.size.width;
					h[i] = target.size.height;
					i++;
				}
				contours.getEntry("x").setDoubleArray(x);
				contours.getEntry("y").setDoubleArray(y);
				contours.getEntry("angle").setDoubleArray(angle);
				contours.getEntry("w").setDoubleArray(w);
				contours.getEntry("h").setDoubleArray(h);

				double[] leftX = new double[pipeline.targetPairs.size()];
				double[] leftY = new double[pipeline.targetPairs.size()];
				double[] leftAngle = new double[pipeline.targetPairs.size()];
				double[] leftW = new double[pipeline.targetPairs.size()];
				double[] leftH = new double[pipeline.targetPairs.size()];
				double[] rightX = new double[pipeline.targetPairs.size()];
				double[] rightY = new double[pipeline.targetPairs.size()];
				double[] rightAngle = new double[pipeline.targetPairs.size()];
				double[] rightW = new double[pipeline.targetPairs.size()];
				double[] rightH = new double[pipeline.targetPairs.size()];
				double[] centerX = new double[pipeline.targetPairs.size()];
				double[] centerY = new double[pipeline.targetPairs.size()];
				i = 0;
				for (VisionTargetPipeline.TargetPair targetPair : pipeline.targetPairs) {
					leftX[i] = targetPair.left.center.x;
					leftY[i] = targetPair.left.center.y;
					leftAngle[i] = targetPair.left.angle;
					leftW[i] = targetPair.left.size.width;
					leftH[i] = targetPair.left.size.height;
					rightX[i] = targetPair.right.center.x;
					rightY[i] = targetPair.right.center.y;
					rightAngle[i] = targetPair.right.angle;
					rightW[i] = targetPair.right.size.width;
					rightH[i] = targetPair.right.size.height;
					centerX[i] = targetPair.center.x;
					centerY[i] = targetPair.center.y;
					i++;
				}
				targetPairs.getEntry("leftX").setDoubleArray(leftX);
				targetPairs.getEntry("leftY").setDoubleArray(leftY);
				targetPairs.getEntry("leftAngle").setDoubleArray(leftAngle);
				targetPairs.getEntry("leftW").setDoubleArray(leftW);
				targetPairs.getEntry("leftH").setDoubleArray(leftH);
				targetPairs.getEntry("rightX").setDoubleArray(rightX);
				targetPairs.getEntry("rightY").setDoubleArray(rightY);
				targetPairs.getEntry("rightAngle").setDoubleArray(rightAngle);
				targetPairs.getEntry("rightW").setDoubleArray(rightW);
				targetPairs.getEntry("rightH").setDoubleArray(rightH);
				targetPairs.getEntry("centerX").setDoubleArray(centerX);
				targetPairs.getEntry("centerY").setDoubleArray(centerY);
			});
			visionThread.start();
		}

		// loop forever
		for (;;) {
			try {
				Thread.sleep(10000);
			} catch (InterruptedException ex) {
				return;
			}
		}
	}
}
