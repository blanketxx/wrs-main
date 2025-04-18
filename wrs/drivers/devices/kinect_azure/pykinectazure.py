import cv2
import ctypes
import platform
import numpy as np
from . import _k4a, _k4a_types, _k4a_record, _k4abt
from .body_tracker import KinectBodyTracker
from .config import Config


class IMUResult(object):
    def __init__(self):
        self.temperature = None
        self.acc_sample = None
        self.acc_timestamp_usec = None
        self.gyro_sample = None
        self.gyro_timestamp_usec = None


class PyKinectAzure(object):

    def __init__(self, lib_path=None):
        if lib_path is None:
            if platform.system().lower() == 'linux':
                lib_path = r'/usr/lib/x86_64-linux-gnu/libk4a.so'
            else:
                lib_path = 'C:\\Program Files\\Azure Kinect SDK v1.4.1\\sdk\\windows-desktop\\amd64\\release\\bin\\k4a.dll'
        self.lib_path = lib_path
        _k4a.k4a.setup_library(lib_path)
        self.k4a = _k4a.k4a()
        self.device_handle = _k4a.k4a_device_t()
        self.capture_handle = _k4a.k4a_capture_t()
        self.config = Config()
        self.imu_sample = _k4a.k4a_imu_sample_t()
        self.cameras_running = False
        self.imu_running = False
        self.recording = False
        # for fast access
        self.device_open()
        self.device_start_cameras()
        self.calibration = self.get_calibration()
        self.transformation_handle = self.transformation_create(self.calibration)

    def update(self):
        # Get capture
        self.device_get_capture()
        # Write capture if recording
        if self.recording:
            self.write_frame()

    def bt_start(self, bodyTrackerModulePath, modelType=_k4abt.K4ABT_DEFAULT_MODEL):
        # Get sensor calibration
        calibration = self.get_calibration()
        # Initialize the body tracker
        self.body_tracker = KinectBodyTracker(bodyTrackerModulePath, calibration, modelType)

    def bt_update(self):
        # Add capture to the body tracker processing queue
        self.body_tracker.enqueue_capture(self.capture_handle)
        # Perform body detection
        self.body_tracker.detectBodies()

    def bt_get_body_segmentation(self):
        # Get the body segmentation image
        body_image = self.image_convert_to_numpy(self.body_tracker.segmented_body_img).astype(np.uint8)
        # Add color to the segmentation based on the id value of each pixel
        body_image_color = np.dstack([cv2.LUT(body_image, _k4abt.body_colors[:, i]) for i in range(3)])
        # Add bod
        return body_image_color

    def bt_project_skeleton(self, skeleton, dest_camera=None):
        if dest_camera is None:
            dest_camera = _k4a.K4A_CALIBRATION_TYPE_DEPTH
        # Project using the calibration of the camera for the image
        position_2d = _k4a.k4a_float2_t()
        valid = ctypes.c_int()
        skeleton2D = _k4abt.k4abt_skeleton2D_t()
        for jointID, joint in enumerate(skeleton.jnts):
            _k4a.VERIFY(self.k4a.k4a_calibration_3d_to_2d(
                self.body_tracker.sensor_calibration,
                joint.position,
                _k4a.K4A_CALIBRATION_TYPE_DEPTH,
                dest_camera,
                position_2d,
                valid),
                "Project skeleton failed")
            skeleton2D.joints2D[jointID].position = position_2d
            skeleton2D.joints2D[jointID].confidence_level = joint.confidence_level
        return skeleton2D

    def device_get_installed_count(self):
        """Gets the number of connected devices
        Parameters:
        None
        Returns:
        int: Number of sensors connected to the PC.
        Remarks:
        This API counts the number of Azure Kinect devices connected to the host PC.
        """
        return int(self.k4a.k4a_device_get_installed_count())

    def device_open(self, index=0):
        """
        Open an Azure Kinect device.
        If successful, k4a_device_open() will return a device handle in the device_handle parameter.
        This handle grants exclusive access to the device and may be used in the other Azure Kinect API calls.
        When done with the device, close the handle with k4a_device_close()
        :param index: int, the index of the device to open, starting with
        :return:
        """
        _k4a.VERIFY(self.k4a.k4a_device_open(index, self.device_handle), "Open K4A Device failed!")

    def device_close(self):
        """
        Closes an Azure Kinect device.
        :return:
        """
        self.k4a.k4a_device_close(self.device_handle)

    def device_get_serialnum(self):
        """
        Get the Azure Kinect device serial number.
        Queries the device for its serial number. If the caller needs to know the size of the serial number to allocate
        memory, the function should be called once with a NULL serial_number to get the needed size in the
        serial_number_size output, and then again with the allocated buffer.
        Only a complete serial number will be returned. If the caller's buffer is too small, the function will return
        ::K4A_BUFFER_RESULT_TOO_SMALL without returning any data in serial_number.
        :return: A return of ::K4A_BUFFER_RESULT_SUCCEEDED means that the serial_number has been filled in. If the buffer is too
                 small the function returns ::K4A_BUFFER_RESULT_TOO_SMALL and the size of the serial number is
                returned in the serial_number_size parameter. All other failures return ::K4A_BUFFER_RESULT_FAILED.
        """
        # First call to get the size of the buffer
        serial_number_size = ctypes.c_size_t()
        result = self.k4a.k4a_device_get_serialnum(self.device_handle, None, serial_number_size)
        if result == _k4a.K4A_BUFFER_RESULT_TOO_SMALL:
            serial_number = ctypes.create_string_buffer(serial_number_size.value)
        _k4a.VERIFY(self.k4a.k4a_device_get_serialnum(self.device_handle, serial_number, serial_number_size),
                    "Read serial number failed!")
        return serial_number.value.decode("utf-8")

    def device_start_cameras(self, device_config=None):
        """
        Starts color and depth camera capture.
        Individual sensors configured to run will now start to stream captured data..
        It is not valid to call k4a_device_start_cameras() a second time on the same k4a_device_t until
        k4a_device_stop_cameras() has been called.
        :param device_config (k4a_device_configuration_t): The configuration we want to run the device in. This can be initialized with ::K4A_DEVICE_CONFIG_INIT_DEFAULT.
        :return None
        """
        if device_config is not None:
            self.config = device_config
        if not self.cameras_running:
            _k4a.VERIFY(self.k4a.k4a_device_start_cameras(self.device_handle, self.config.current_config),
                        "Start K4A cameras failed!")
            self.cameras_running = True

    def device_stop_cameras(self):
        """
        Stops the color and depth camera capture.
        The streaming of individual sensors stops as a result of this call. Once called, k4a_device_start_cameras() may
        be called again to resume sensor streaming.
        """
        if self.cameras_running:
            self.k4a.k4a_device_stop_cameras(self.device_handle)
            self.cameras_running = False

    def device_start_imu(self):
        """
        Starts the IMU sample stream.
        Call this API to start streaming IMU data. It is not valid to call this function a second time on the same
        k4a_device_t until k4a_device_stop_imu() has been called.
        This function is dependent on the state of the cameras. The color or depth camera must be started before the IMU.
        K4A_RESULT_FAILED will be returned if one of the cameras is not running.
        """
        if self.cameras_running:
            if not self.imu_running:
                _k4a.VERIFY(self.k4a.k4a_device_start_imu(self.device_handle), "Start K4A IMU failed!")
                self.imu_running = True
        else:
            print("\nTurn on cameras before running IMU.\n")

    def device_stop_imu(self):
        """
        Stops the IMU capture.
        The streaming of the IMU stops as a result of this call. Once called, k4a_device_start_imu() may
        be called again to resume sensor streaming, so long as the cameras are running.
        This function may be called while another thread is blocking in k4a_device_get_imu_sample().
        Calling this function while another thread is in that function will result in that function returning a failure.
        """
        if self.imu_running:
            self.k4a.k4a_device_stop_imu(self.device_handle)
            self.imu_running = False

    def device_get_capture(self, timeout_in_ms=_k4a.K4A_WAIT_INFINITE):
        """
        Reads a sensor capture.
        :param timeout_in_ms (int):Specifies the time in milliseconds the function should block waiting for the capture. If set to 0, the function will
                            return without blocking. Passing a value of #K4A_WAIT_INFINITE will block indefinitely until data is available, the
                            device is disconnected, or another error occurs.
        :return
        Gets the next capture in the streamed sequence of captures from the camera. If a new capture is not currently
        available, this function will block until the timeout is reached. The SDK will buffer at least two captures worth
        of data before dropping the oldest capture. Callers needing to capture all data need to ensure they read the data as
        fast as the data is being produced on average.
        Upon successfully reading a capture this function will return success and populate capture.
        If a capture is not available in the configured timeout_in_ms, then the API will return ::K4A_WAIT_RESULT_TIMEOUT.
        """
        if self.cameras_running:
            _k4a.VERIFY(self.k4a.k4a_device_get_capture(self.device_handle, self.capture_handle, timeout_in_ms),
                        "Get capture failed!")

    def device_get_imu_sample(self, timeout_in_ms=_k4a.K4A_WAIT_INFINITE):
        """
        Reads an IMU sample.
        Gets the next sample in the streamed sequence of IMU samples from the device. If a new sample is not currently
        available, this function will block until the timeout is reached. The API will buffer at least two camera capture
        intervals worth of samples before dropping the oldest sample. Callers needing to capture all data need to ensure they
        read the data as fast as the data is being produced on average.
        Upon successfully reading a sample this function will return success and populate imu_sample.
        If a sample is not available in the configured timeout_in_ms, then the API will return ::K4A_WAIT_RESULT_TIMEOUT.
        :param timeout_in_ms (int):Specifies the time in milliseconds the function should block waiting for the capture. If set to 0, the function will
                            return without blocking. Passing a value of #K4A_WAIT_INFINITE will block indefinitely until data is available, the
                            device is disconnected, or another error occurs.
        :return
        """
        if self.imu_running:
            _k4a.VERIFY(self.k4a.k4a_device_get_imu_sample(self.device_handle, self.imu_sample, timeout_in_ms),
                        "Get IMU failed!")

    def device_get_calibration(self, depth_mode, color_resolution, calibration):
        """
        Get the camera calibration for the entire Azure Kinect device.
        The calibration represents the data needed to transform between the camera views and may be
        different for each operating depth_mode and color_resolution the device is configured to operate in.
        The calibration output is used as input to all calibration and transformation functions.
        :param depth_mode(k4a_depth_mode_t): Mode in which depth camera is operated.
        :param color_resolution(k4a_color_resolution_t): Resolution in which color camera is operated.
        :param calibration(k4a_calibration_t): Location to write the calibration
        :return K4A_RESULT_SUCCEEDED if calibration was successfully written. ::K4A_RESULT_FAILED otherwise.
        """
        _k4a.VERIFY(self.k4a.k4a_device_get_calibration(self.device_handle,
                                                        depth_mode,
                                                        color_resolution,
                                                        calibration),
                    "Get calibration failed!")

    def capture_get_color_image(self):
        """
        Get the color image associated with the given capture.
        Call this function to access the color image part of this capture. Release the ref k4a_image_t with
        k4a_image_release();
        :return: k4a_image_t: Handle to the Image
        """
        return self.k4a.k4a_capture_get_color_image(self.capture_handle)

    def capture_get_depth_image(self):
        """
        Get the depth image associated with the given capture.
        Call this function to access the depth image part of this capture. Release the k4a_image_t with
        k4a_image_release();
        :return: k4a_image_t: Handle to the Image
        """
        return self.k4a.k4a_capture_get_depth_image(self.capture_handle)

    def capture_get_ir_image(self):
        """
        Get the IR image associated with the given capture.
        Call this function to access the IR image part of this capture. Release the k4a_image_t with
        k4a_image_release();
        :return: k4a_image_t: Handle to the Image
        """
        return self.k4a.k4a_capture_get_ir_image(self.capture_handle)

    def image_create(self, image_format, width_pixels, height_pixels, stride_bytes, image_handle):
        """
        Create an image.
        This function is used to create images of formats that have consistent stride. The function is not suitable for
        compressed formats that may not be represented by the same number of bytes per line.
        For most image formats, the function will allocate an image buffer of size height_pixels * stride_bytes.
        Buffers #K4A_IMAGE_FORMAT_COLOR_NV12 format will allocate an additional height_pixels / 2 set of lines (each of
        stride_bytes). This function cannot be used to allocate #K4A_IMAGE_FORMAT_COLOR_MJPG buffers.
        :param image_format: k4a_image_format_t, the format of the image that will be stored in this image container.
        :param width_pixels: int, width in pixels
        :param height_pixels: int, height in pixels
        :param stride_bytes: int, the number of bytes per horizontal line of the image.
                             If set to 0, the stride will be set to the minimum size given the format and width_pixels.
        :param image_handle: k4a_image_t, pointer to store image handle in.
        :return: #K4A_RESULT_SUCCEEDED on success. Errors are indicated with #K4A_RESULT_FAILED.
        """
        _k4a.VERIFY(self.k4a.k4a_image_create(image_format, width_pixels, height_pixels, stride_bytes, image_handle),
                    "Create image failed!")

    def image_get_buffer(self, image_handle):
        """
        Get the image buffer.
        Use this buffer to access the raw image data.
        :param image_handle (k4a_image_t): Handle to the Image
        :return ctypes.POINTER(ctypes.c_uint8): The function will return NULL if there is an error, and will normally return a pointer to the image buffer.
                Since all k4a_image_t instances are created with an image buffer, this function should only return NULL if the
                image_handle is invalid.
        """
        return self.k4a.k4a_image_get_buffer(image_handle)

    def image_get_size(self, image_handle):
        """
        Get the image buffer size.
        Use this function to know what the size of the image buffer is returned by k4a_image_get_buffer().
        :param image_handle (k4a_image_t): Handle to the Image
        :return int: The function will return 0 if there is an error, and will normally return the image size.
                Since all k4a_image_t instances are created with an image buffer, this function should only return 0 if the
                image_handle is invalid.
        """
        return int(self.k4a.k4a_image_get_size(image_handle))

    def image_get_format(self, image_handle):
        """
        Get the format of the image.
        Use this function to determine the format of the image buffer.
        :param image_handle (k4a_image_t): Handle to the Image
        :return int: This function is not expected to fail, all k4a_image_t's are created with a known format. If the
                image_handle is invalid, the function will return ::K4A_IMAGE_FORMAT_CUSTOM.
        """
        return int(self.k4a.k4a_image_get_format(image_handle))

    def image_get_width_pixels(self, image_handle):
        """
        Get the image width in pixels.
        int: This function is not expected to fail, all k4a_image_t's are created with a known width. If the part
        image_handle is invalid, the function will return 0.
        :param image_handle (k4a_image_t): Handle to the Image
        :return
        """
        return int(self.k4a.k4a_image_get_width_pixels(image_handle))

    def image_get_height_pixels(self, image_handle):
        """
        Get the image height in pixels.
        :param image_handle (k4a_image_t): Handle to the Image
        :return int: This function is not expected to fail, all k4a_image_t's are created with a known height. If the part
                image_handle is invalid, the function will return 0.
        """
        return int(self.k4a.k4a_image_get_height_pixels(image_handle))

    def image_get_stride_bytes(self, image_handle):
        """
        Get the image stride in bytes.
        :param image_handle (k4a_image_t): Handle to the Image
        :return int: This function is not expected to fail, all k4a_image_t's are created with a known stride. If the
                image_handle is invalid, or the image's format does not have a stride, the function will return 0.
        """
        return int(self.k4a.k4a_image_get_stride_bytes(image_handle))

    def transformation_create(self, calibration):
        """
        Get handle to transformation handle.
        The transformation handle is used to transform images from the coordinate system of one camera into the other. Each
        transformation handle requires some pre-computed resources to be allocated, which are retained until the handle is
        destroyed.
        The transformation handle must be destroyed with k4a_transformation_destroy() when it is no longer to be used.
        :param calibration(k4a_calibration_t): A calibration structure obtained by k4a_device_get_calibration().
        :return k4a_transformation_t: A transformation handle. A NULL is returned if creation fails.
        """
        return self.k4a.k4a_transformation_create(calibration)

    def transformation_destroy(self, transformation_handle):
        """
        Destroy transformation handle.
        :param transformation_handle(k4a_transformation_t): Transformation handle to destroy.
        :return
        """
        self.k4a.k4a_transformation_destroy(transformation_handle)

    def transform_depth_image_to_point_cloud(self, depth_image_handle: _k4a.k4a_image_t):
        """
        Transforms the depth map to point clouds
        :param depth_image_handle (k4a_image_t): Handle to the Image
        :return point_cloud numpy array
        author: weiwei
        date: 20210708
        """
        point_cloud = _k4a_types.k4a_image_t()
        self.image_create(
            _k4a_types.K4A_IMAGE_FORMAT_CUSTOM,
            self.image_get_width_pixels(depth_image_handle),
            self.image_get_height_pixels(depth_image_handle),
            self.image_get_width_pixels(depth_image_handle) * 6,
            point_cloud
        )
        _k4a.VERIFY(self.k4a.k4a_transformation_depth_image_to_point_cloud(
            self.transformation_handle,
            depth_image_handle,
            _k4a_types.K4A_CALIBRATION_TYPE_DEPTH,
            point_cloud
        ), "Error Occur When Make Point Cloud")
        # convert to point cloud
        buffer_pointer = self.image_get_buffer(point_cloud)
        image_size = self.image_get_size(point_cloud)
        image_width = self.image_get_width_pixels(point_cloud)
        image_height = self.image_get_height_pixels(point_cloud)
        buffer_array = np.ctypeslib.as_array(buffer_pointer, shape=(image_size,))
        return np.frombuffer(buffer_array, dtype=np.int16).reshape(image_height * image_width, 3) / 1000

    def transformation_depth_image_to_color_camera(self, transformation_handle, input_depth_image_handle,
                                                   transformed_depth_image_handle):
        """
        Transforms the depth map into the geometry of the color camera.
        transformed_depth_image must have a width and height matching the width and height of the color camera in the mode
        specified by the k4a_calibration_t used to create the transformation_handle with k4a_transformation_create().
        This produces a depth image for which each pixel matches the corresponding pixel coordinates of the color camera.
        :param transformation_handle (k4a_transformation_t): Transformation handle.
               input_depth_image_handle (k4a_image_t): Handle to input depth image.
               transformed_depth_image_handle (k4a_image_t): Handle to output transformed depth image.
        :return K4A_RESULT_SUCCEEDED if transformed_depth_image was successfully written and ::K4A_RESULT_FAILED otherwise.
        """
        _k4a.VERIFY(
            self.k4a.k4a_transformation_depth_image_to_color_camera(transformation_handle,
                                                                    input_depth_image_handle,
                                                                    transformed_depth_image_handle),
            "Transformation from depth to color failed!")

    def transformation_color_image_to_depth_camera(self, transformation_handle,
                                                   input_depth_image_handle,
                                                   input_color_image_handle,
                                                   transformed_color_image_handle):
        """
        Transforms the color image into the geometry of the depth camera.
        transformed_color_image must have a width and height matching the width and height of the depth camera in the mode
        specified by the k4a_calibration_t used to create the transformation_handle with k4a_transformation_create().
        This produces a depth image for which each pixel matches the corresponding pixel coordinates of the depth camera.
        :param transformation_handle (k4a_transformation_t): Transformation handle.
               input_color_image_handle (k4a_image_t): Handle to input color image.
               transformed_color_image_handle (k4a_image_t): Handle to output transformed color image.
        :return K4A_RESULT_SUCCEEDED if transformed_depth_image was successfully written and ::K4A_RESULT_FAILED otherwise.
        """
        _k4a.VERIFY(
            self.k4a.k4a_transformation_color_image_to_depth_camera(transformation_handle,
                                                                    input_depth_image_handle,
                                                                    input_color_image_handle,
                                                                    transformed_color_image_handle),
            "Transformation from color to depth failed!")

    def image_convert_to_numpy(self, image_handle):
        """
        Get the image data as a numpy array
        :param image_handle (k4a_image_t): Handle to the Image
        :return numpy.ndarray: Numpy array with the image data
        """
        # Get the pointer to the buffer containing the image data
        buffer_pointer = self.image_get_buffer(image_handle)
        # Get the size of the buffer
        image_size = self.image_get_size(image_handle)
        image_width = self.image_get_width_pixels(image_handle)
        image_height = self.image_get_height_pixels(image_handle)
        # Get the image format
        image_format = self.image_get_format(image_handle)
        # Read the data in the buffer
        buffer_array = np.ctypeslib.as_array(buffer_pointer, shape=(image_size,))
        # Parse buffer based on image format
        if image_format == _k4a.K4A_IMAGE_FORMAT_COLOR_MJPG:
            return cv2.imdecode(np.frombuffer(buffer_array, dtype=np.uint8), -1)
        elif image_format == _k4a.K4A_IMAGE_FORMAT_COLOR_NV12:
            yuv_image = np.frombuffer(buffer_array, dtype=np.uint8).reshape(int(image_height * 1.5), image_width)
            return cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_NV12)
        elif image_format == _k4a.K4A_IMAGE_FORMAT_COLOR_YUY2:
            yuv_image = np.frombuffer(buffer_array, dtype=np.uint8).reshape(image_height, image_width, 2)
            return cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_YUY2)
        elif image_format == _k4a.K4A_IMAGE_FORMAT_COLOR_BGRA32:
            return np.frombuffer(buffer_array, dtype=np.uint8).reshape(image_height, image_width, 4)
        elif image_format == _k4a.K4A_IMAGE_FORMAT_DEPTH16:
            return np.frombuffer(buffer_array, dtype="<u2").reshape(image_height,
                                                                    image_width)  # little-endian 16 bits unsigned Depth data
        elif image_format == _k4a.K4A_IMAGE_FORMAT_IR16:
            return np.frombuffer(buffer_array, dtype="<u2").reshape(image_height,
                                                                    image_width)  # little-endian 16 bits unsigned IR data. For more details see: https://microsoft.github.io/Azure-Kinect-Sensor-SDK/release/1.2.x/namespace_microsoft_1_1_azure_1_1_kinect_1_1_sensor_a7a3cb7a0a3073650bf17c2fef2bfbd1b.html
        elif image_format == _k4a.K4A_IMAGE_FORMAT_CUSTOM8:
            return np.frombuffer(buffer_array, dtype="<u1").reshape(image_height, image_width)

    def transform_depth_to_color(self, input_depth_image_handle, color_image_handle):
        calibration = _k4a.k4a_calibration_t()
        # Get desired image format
        image_format = self.image_get_format(input_depth_image_handle)
        image_width = self.image_get_width_pixels(color_image_handle)
        image_height = self.image_get_height_pixels(color_image_handle)
        image_stride = 0
        # Get the camera calibration
        self.device_get_calibration(self.config.depth_mode, self.config.color_resolution, calibration)
        # Create the image handle
        transformed_depth_image_handle = _k4a.k4a_image_t()
        self.image_create(image_format, image_width, image_height, image_stride, transformed_depth_image_handle)
        # Transform the depth image to the color image format
        self.transformation_depth_image_to_color_camera(self.transformation_handle, input_depth_image_handle,
                                                        transformed_depth_image_handle)
        # Get transformed image data
        transformed_image = self.image_convert_to_numpy(transformed_depth_image_handle)
        return transformed_image

    def transform_color_to_depth(self, input_color_image_handle, input_depth_image_handle):
        """
        author: weiwei
        date: 20210708
        """
        # Get desired image format
        image_format = _k4a_types.K4A_IMAGE_FORMAT_COLOR_BGRA32
        image_width = self.image_get_width_pixels(input_depth_image_handle)
        image_height = self.image_get_height_pixels(input_depth_image_handle)
        image_stride = 0

        transformed_color_image_handle = _k4a.k4a_image_t()
        self.image_create(image_format, image_width, image_height, image_stride, transformed_color_image_handle)
        # Transform the color image to the depth image format
        self.transformation_color_image_to_depth_camera(self.transformation_handle,
                                                        input_depth_image_handle,
                                                        input_color_image_handle,
                                                        transformed_color_image_handle)
        # Get transformed image data
        transformed_image = self.image_convert_to_numpy(transformed_color_image_handle)

        return transformed_image

    def transform_color_xy_to_pcd_xyz(self, input_color_image_handle, input_depth_image_handle, color_xy):
        """
        :param color_xy np.array([x,y])
        author:weiwei
        date: 20210712
        """
        # image_format = _k4a_types.K4A_IMAGE_FORMAT_COLOR_BGRA32
        # image_width = self.image_get_width_pixels(input_color_image_handle)
        # image_height = self.image_get_height_pixels(input_color_image_handle)
        # image_stride = 0
        # tmp_color_image_handle = _k4a.k4a_image_t()
        # self.image_create(image_format, image_width, image_height, image_stride, tmp_color_image_handle)
        calibration = self.get_calibration()
        position_2d = _k4a.k4a_float2_t()
        position_2d.xy.x = color_xy[0]
        position_2d.xy.y = color_xy[1]
        depth_on_color = self.transform_depth_to_color(input_depth_image_handle, input_color_image_handle)
        source_depth_mm = depth_on_color[color_xy[1], color_xy[0]]
        target_point3d_mm = _k4a.k4a_float3_t()
        valid = ctypes.c_int()
        _k4a.VERIFY(
            self.k4a.k4a_calibration_2d_to_3d(calibration,
                                              position_2d,
                                              source_depth_mm,
                                              _k4a.K4A_CALIBRATION_TYPE_COLOR,
                                              _k4a.K4A_CALIBRATION_TYPE_DEPTH,
                                              target_point3d_mm,
                                              valid),
            "Transformation from color to depth failed!")
        xyz = target_point3d_mm.xyz
        return np.array([xyz.x, xyz.y, xyz.z]) * 1e-3

    def get_calibration(self):
        calibration = _k4a.k4a_calibration_t()
        self.device_get_calibration(self.config.depth_mode, self.config.color_resolution, calibration)
        return calibration

    def get_depth_intrinsics(self):
        """
        :return intrinsic matrix, distortion, rvecs, tvecs
        author: weiwei
        date: 20210708
        """
        calibration = self.get_calibration()
        mtx = np.eye(3)
        mtx[0, 0] = calibration.depth_camera_calibration.intrinsics.parameters.param.fx
        mtx[1, 1] = calibration.depth_camera_calibration.intrinsics.parameters.param.fy
        mtx[0, 2] = calibration.depth_camera_calibration.intrinsics.parameters.param.cx
        mtx[1, 2] = calibration.depth_camera_calibration.intrinsics.parameters.param.cy
        dist = np.array([calibration.depth_camera_calibration.intrinsics.parameters.param.k1,
                         calibration.depth_camera_calibration.intrinsics.parameters.param.k2,
                         calibration.depth_camera_calibration.intrinsics.parameters.param.p1,
                         calibration.depth_camera_calibration.intrinsics.parameters.param.p2,
                         calibration.depth_camera_calibration.intrinsics.parameters.param.k3])
        return mtx, dist, np.zeros(3), np.zeros(3)

    def get_color_intrinsics(self):
        """
        :return intrinsic matrix, distortion, rvecs, tvecs
        author: weiwei
        date: 20210708
        """
        calibration = self.get_calibration()
        mtx = np.eye(3)
        mtx[0, 0] = calibration.color_camera_calibration.intrinsics.parameters.param.fx
        mtx[1, 1] = calibration.color_camera_calibration.intrinsics.parameters.param.fy
        mtx[0, 2] = calibration.color_camera_calibration.intrinsics.parameters.param.cx
        mtx[1, 2] = calibration.color_camera_calibration.intrinsics.parameters.param.cy
        dist = np.array([calibration.color_camera_calibration.intrinsics.parameters.param.k1,
                         calibration.color_camera_calibration.intrinsics.parameters.param.k2,
                         calibration.color_camera_calibration.intrinsics.parameters.param.p1,
                         calibration.color_camera_calibration.intrinsics.parameters.param.p2,
                         calibration.color_camera_calibration.intrinsics.parameters.param.k3])
        return [mtx, dist, np.zeros(3), np.zeros(3)]

    def image_release(self, image_handle):
        """
        Remove a reference from the k4a_image_t.
        References manage the lifetime of the object. When the references reach zero the object is destroyed. A caller must
        not access the object after its reference is released.
        :param image_handle: k4a_image_t, Handle to the Image
        :return:
        """
        self.k4a.k4a_image_release(image_handle)

    def capture_release(self):
        """
        Release a capture.
        Call this function when finished using the capture.
        :return:
        """
        self.k4a.k4a_capture_release(self.capture_handle)

    def get_imu_sample(self, timeout_in_ms=_k4a.K4A_WAIT_INFINITE):
        """
        :param timeout_in_ms:
        :return:
        """
        # Get the sample from the device
        self.device_get_imu_sample(timeout_in_ms)
        # Read the raw data from the buffer pointer
        buffer_array = np.array(np.ctypeslib.as_array(self.imu_sample, shape=(_k4a.IMU_SAMPLE_SIZE,)).tolist())
        imu_results = IMUResult()
        imu_results.temperature = buffer_array[0]
        imu_results.acc_sample = buffer_array[1][1]
        imu_results.acc_timestamp_usec = buffer_array[2]
        imu_results.gyro_sample = buffer_array[3][1]
        imu_results.gyro_timestamp_usec = buffer_array[4]
        return imu_results

    def start_recording(self, filepath="output.mkv"):
        self.record = record.Record(self.lib_path, self.device_handle, self.config.current_config, filepath)
        self.recording = True

    def stop_recording(self):
        self.record = None
        self.recording = False

    def write_frame(self):
        self.record.write_capture(self.capture_handle)
