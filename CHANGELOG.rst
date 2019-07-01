^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flir_boson_usb
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.1 (2019-07-01)
------------------
* Merge pull request `#3 <https://github.com/astuff/flir_boson_usb/issues/3>`_ from valgur/patch-1
* Fix minor issues detected by catkin_lint
* Fixing installation of nodelet_plugins.xml.
* Contributors: Joe Driscoll, Joshua Whitley, Martin Valgur, Sam Rustan

1.2.0 (2019-01-24)
------------------
* Merge pull request `#2 <https://github.com/astuff/flir_boson_usb/issues/2>`_ from astuff/feat/add_camera_info_manager
  Feat/add camera info manager
* Defaulting to 60 fps everywhere.
* Swapping boost shared pointers for std shared pointers.
* More expressive error messages.
* Adding frame_rate parameter.
* Adding frame_id param and actually making it do something useful.
* Adding launch file with rectification.
* Cleaning up example file and launch file. Added namespace to launch.
* Adding example Boson_640.yaml calibration file.
* Adding camera_info_manager for image rectification.
* Contributors: Joshua Whitley, Rinda Gunjala

1.1.2 (2019-01-23)
------------------
* Found bug in device path parameter.
* Contributors: Joshua Whitley

1.1.1 (2019-01-21)
------------------
* Temporarily removing camera_info_manager.
* Contributors: Joshua Whitley

1.1.0 (2019-01-21)
------------------
* Modified install of launch folder.
* Added roslint and cleaned up based on suggestions.
* Converted driver to BosonCamera nodelet.
* Contributors: Joshua Whitley

1.0.0 (2018-12-13)
------------------
* Adding LICENSE.
* Adding status badge to README.
* Adding launch file.
* Initial commit.
* Contributors: Joshua Whitley
