"""
Gelsight Shear Force Measurement Demo

This script extends the marker tracking functionality to measure shear forces
applied to the Gelsight sensor surface. It tracks marker displacements and
computes shear strain and force based on the deformation field.
"""

import os

os.environ["KIVY_NO_ARGS"] = "1"
from kivy.app import App
from kivy.clock import Clock
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.image import Image
from kivy.uix.widget import Widget
from kivy.uix.label import Label
from kivy.uix.slider import Slider
from kivy.uix.button import Button
from kivy.core.window import Window
from kivy.config import Config
from kivy.metrics import dp
from kivy.graphics.texture import Texture

from utilities.gelsightmini import GelSightMini
from utilities.image_processing import add_fps_count_overlay, rescale
from utilities.ui_components import ConnectingOverlay, FileChooserPopup, TopBar
from utilities.logger import log_message
from config import ConfigModel
import cv2
import numpy as np
from utilities.marker_tracker import MarkerTracker
from datetime import datetime
from typing import Optional, Tuple

Config.set("input", "mouse", "mouse,multitouch_on_demand")


class ShearForceLogger:
    """
    Logger for shear force data including marker positions, displacements,
    strains, and computed shear forces.
    """

    def __init__(self):
        self.frames_data = []
        self.reference_positions = None
        
    def set_reference(self, positions: np.ndarray):
        """Set reference positions for displacement calculation"""
        self.reference_positions = positions.copy()
        
    def add_frame(self, positions: np.ndarray, shear_force_x: float, shear_force_y: float, 
                 shear_magnitude: float):
        """Add frame data including positions and computed forces"""
        if self.reference_positions is None:
            return
            
        displacements = positions - self.reference_positions
        
        frame_data = {
            'timestamp': datetime.now().isoformat(),
            'positions': positions.copy(),
            'displacements': displacements,
            'shear_force_x': shear_force_x,
            'shear_force_y': shear_force_y, 
            'shear_magnitude': shear_magnitude
        }
        self.frames_data.append(frame_data)
        
    def save_data(self, folder: str = "."):
        """Save collected shear force data"""
        if not self.frames_data:
            log_message("No shear force data to save.")
            return
            
        date_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"shear_force_data_{date_str}"
        
        # Save as numpy file
        np_data = {
            'timestamps': [frame['timestamp'] for frame in self.frames_data],
            'positions': np.array([frame['positions'] for frame in self.frames_data]),
            'displacements': np.array([frame['displacements'] for frame in self.frames_data]),
            'shear_force_x': np.array([frame['shear_force_x'] for frame in self.frames_data]),
            'shear_force_y': np.array([frame['shear_force_y'] for frame in self.frames_data]),
            'shear_magnitude': np.array([frame['shear_magnitude'] for frame in self.frames_data])
        }
        
        numpy_filepath = os.path.join(folder, f"{filename}.npy")
        np.save(numpy_filepath, np_data)
        log_message(f"Saved shear force data: {numpy_filepath}")
        
        # Save as CSV for easy analysis
        csv_filepath = os.path.join(folder, f"{filename}.csv")
        with open(csv_filepath, 'w') as f:
            f.write("timestamp,shear_force_x,shear_force_y,shear_magnitude\n")
            for frame in self.frames_data:
                f.write(f"{frame['timestamp']},{frame['shear_force_x']:.4f},"
                       f"{frame['shear_force_y']:.4f},{frame['shear_magnitude']:.4f}\n")
        log_message(f"Saved shear force CSV: {csv_filepath}")
        
        self.frames_data = []


class ShearForceCalculator:
    """
    Calculator for shear forces based on marker displacement fields.
    
    This class implements a simplified shear force calculation based on:
    1. Marker displacement tracking
    2. Shear strain estimation from displacement gradients
    3. Force calculation using material properties
    """
    
    def __init__(self, gel_thickness: float = 0.004, shear_modulus: float = 50000):
        """
        Initialize shear force calculator.
        
        Args:
            gel_thickness (float): Thickness of gel layer in meters (default: 4mm)
            shear_modulus (float): Shear modulus of gel in Pa (default: 50kPa)
        """
        self.gel_thickness = gel_thickness
        self.shear_modulus = shear_modulus
        self.reference_positions = None
        
        # Pixel to physical conversion (approximate, sensor specific)
        # This should be calibrated for your specific sensor
        self.pixel_to_meter = 20e-6  # 20 micrometers per pixel (typical)
        
    def set_reference(self, positions: np.ndarray):
        """Set reference positions for zero strain state"""
        self.reference_positions = positions.copy()
        
    def calculate_shear_force(self, current_positions: np.ndarray) -> Tuple[float, float, float]:
        """
        Calculate shear force from marker positions.
        
        Args:
            current_positions: Current marker positions (N, 2)
            
        Returns:
            Tuple of (shear_force_x, shear_force_y, shear_magnitude) in Newtons
        """
        if self.reference_positions is None:
            return 0.0, 0.0, 0.0
            
        # Calculate displacements in pixels
        displacements = current_positions - self.reference_positions
        
        # Convert to physical units (meters)
        displacements_m = displacements * self.pixel_to_meter
        
        # Simple average displacement as proxy for overall shear
        # In a more sophisticated approach, you would fit a displacement field
        # and calculate strain tensor components
        avg_displacement_x = np.mean(displacements_m[:, 0])
        avg_displacement_y = np.mean(displacements_m[:, 1])
        
        # Simple shear strain calculation (displacement / thickness)
        # This is a simplified model - real shear strain would involve gradients
        shear_strain_x = avg_displacement_x / self.gel_thickness
        shear_strain_y = avg_displacement_y / self.gel_thickness
        
        # Shear stress = shear_modulus * shear_strain
        shear_stress_x = self.shear_modulus * shear_strain_x
        shear_stress_y = self.shear_modulus * shear_strain_y
        
        # Estimate contact area (this should be measured or estimated from sensor)
        # Using a typical tactile sensor area
        contact_area = 0.0001  # 1 cm² in m²
        
        # Force = stress * area
        shear_force_x = shear_stress_x * contact_area
        shear_force_y = shear_stress_y * contact_area
        shear_magnitude = np.sqrt(shear_force_x**2 + shear_force_y**2)
        
        # Debug logging (can be disabled in production)
        if np.max(np.abs(displacements)) > 1:  # Log when displacement > 1 pixel
            from utilities.logger import log_message
            log_message(f"Force calc: disp={np.mean(np.abs(displacements)):.2f}px, "
                       f"force_mag={shear_magnitude:.8f}N")
        
        return shear_force_x, shear_force_y, shear_magnitude


class GelsightShearForce(App):
    def __init__(self, config: ConfigModel, **kwargs):
        super().__init__(**kwargs)

        self.config = config
        self.cam_stream = GelSightMini(
            target_width=self.config.camera_width,
            target_height=self.config.camera_height,
            border_fraction=self.config.border_fraction,
        )

    def build(self):
        self.title = "Gelsight Shear Force Measurement"
        self.loading_overlay = None

        root = BoxLayout(orientation="vertical")
        # Top bar with device selection
        self.top_bar = TopBar(on_device_selected_callback=self.restart_camera_stream)
        root.add_widget(self.top_bar)

        # Create ShearForceViewWidget
        self.shear_force_view = ShearForceViewWidget(main_app=self)
        root.add_widget(self.shear_force_view)

        return root

    def show_overlay(self, message):
        if not self.loading_overlay:
            self.loading_overlay = ConnectingOverlay(message=message)
            self.loading_overlay.open()

    def hide_overlay(self):
        if self.loading_overlay:
            self.loading_overlay.dismiss()
            self.loading_overlay = None

    def restart_camera_stream(self, device_index):
        self.cam_stream.select_device(device_index)
        from kivy.clock import Clock

        Clock.schedule_once(lambda dt: self.finish_device_selection(), 0)

    def finish_device_selection(self):
        self.hide_overlay()
        self.shear_force_view.start()


class ShearForceViewWidget(BoxLayout):
    def __init__(self, main_app: GelsightShearForce, **kwargs):
        super().__init__(orientation="vertical", **kwargs)
        self.main_app = main_app
        self.initialized: bool = False
        self.is_logging_data: bool = False
        self.data_folder_path: str = os.path.join(os.path.expanduser("~"), "Desktop")
        
        # Initialize shear force components
        self.shear_force_calculator = ShearForceCalculator()
        self.shear_force_logger = ShearForceLogger()
        
        # Current shear force values
        self.current_shear_x = 0.0
        self.current_shear_y = 0.0
        self.current_shear_magnitude = 0.0
        
        # Create UI elements:
        self.image_widget = Image()
        self.add_widget(self.image_widget)

        # Shear force display
        force_display_layout = BoxLayout(
            orientation="horizontal", size_hint_y=None, height=dp(80)
        )
        
        # Force values display
        force_values_layout = BoxLayout(orientation="vertical", size_hint=(0.7, 1))
        self.force_x_label = Label(
            text="Shear Force X: -- N (Set Reference First)", 
            size_hint_y=None, height=dp(25),
            halign="left"
        )
        self.force_y_label = Label(
            text="Shear Force Y: -- N (Set Reference First)", 
            size_hint_y=None, height=dp(25),
            halign="left"
        )
        self.force_mag_label = Label(
            text="Magnitude: -- N (Set Reference First)", 
            size_hint_y=None, height=dp(25),
            halign="left"
        )
        force_values_layout.add_widget(self.force_x_label)
        force_values_layout.add_widget(self.force_y_label)
        force_values_layout.add_widget(self.force_mag_label)
        force_display_layout.add_widget(force_values_layout)
        
        # Calibration button
        self.calibrate_btn = Button(
            text="Set Reference\n(Zero Force)", 
            size_hint=(0.3, 1)
        )
        self.calibrate_btn.bind(on_press=self.set_reference)
        force_display_layout.add_widget(self.calibrate_btn)
        
        self.add_widget(force_display_layout)

        # Zoom layout
        zoom_layout = BoxLayout(
            orientation="horizontal", size_hint_y=None, height=dp(40)
        )
        zoom_layout.add_widget(
            Label(text="Zoom:", size_hint=(None, None), size=(dp(60), dp(40)))
        )
        self.zoom_slider = Slider(min=0.5, max=3.0, value=1.0)
        self.zoom_slider.bind(value=self.on_zoom_value_change)
        zoom_layout.add_widget(self.zoom_slider)
        self.zoom_label = Label(
            text="1.0x", size_hint=(None, None), size=(dp(60), dp(40))
        )
        zoom_layout.add_widget(self.zoom_label)
        self.add_widget(zoom_layout)

        # Folder selection layout
        folder_layout = BoxLayout(
            orientation="horizontal",
            size_hint_y=None,
            height=dp(40),
            spacing=dp(80),
            padding=[dp(10)] * 4,
        )
        self.data_folder_btn = Button(
            text="Data Folder", size_hint=(None, None), size=(dp(180), dp(30))
        )
        self.data_folder_btn.bind(on_press=self.open_screenshot_folder_choice)
        folder_layout.add_widget(self.data_folder_btn)
        self.data_folder_label = Label(
            text=f"{os.path.join(os.path.expanduser('~'), 'Desktop')}",
            size_hint=(None, None),
            height=dp(30),
            width=dp(400),
            halign="left",
            valign="middle",
        )
        folder_layout.add_widget(self.data_folder_label)
        self.add_widget(folder_layout)

        # Capture controls layout
        capture_layout = BoxLayout(
            orientation="horizontal",
            size_hint_y=None,
            height=dp(40),
            spacing=dp(20),
            padding=[dp(10)] * 4,
        )
        self.screenshot_btn = Button(
            text="Save Image", size_hint=(None, None), size=(dp(180), dp(30))
        )
        self.screenshot_btn.bind(on_press=self.take_screenshot)
        capture_layout.add_widget(self.screenshot_btn)

        self.record_data_btn = Button(
            text="Start Recording", size_hint=(None, None), size=(dp(180), dp(30))
        )
        self.record_data_btn.bind(on_press=self.toggle_recording)
        capture_layout.add_widget(self.record_data_btn)

        self.reset_tracking = Button(
            text="Reset Tracking", size_hint=(None, None), size=(dp(180), dp(30))
        )
        self.reset_tracking.bind(on_press=self.on_reset_tracking)
        capture_layout.add_widget(self.reset_tracking)

        self.add_widget(capture_layout)
        self.add_widget(Widget(size_hint_y=None, height=dp(10)))

        self.event = None

        # Bind key events for shortcuts
        Window.bind(on_key_down=self.on_key_down)

    def on_zoom_value_change(self, instance, value):
        self.zoom_label.text = f"{value:.1f}x"

    def on_reset_tracking(self, instance=None):
        self.initialized = False
        self.shear_force_calculator.reference_positions = None
        self.shear_force_logger.reference_positions = None

    def set_reference(self, instance=None):
        """Set current marker positions as reference (zero force) state"""
        if hasattr(self, 'p0') and self.p0 is not None:
            current_positions = self.p0.reshape(-1, 2)
            self.shear_force_calculator.set_reference(current_positions)
            self.shear_force_logger.set_reference(current_positions)
            log_message(f"Reference positions set for shear force calculation with {len(current_positions)} markers")
            
            # Update button text to show reference is set
            self.calibrate_btn.text = "Reset Reference\n(Zero Force)"
        else:
            log_message("Cannot set reference - no marker tracking data available yet")

    def start(self):
        # Start the camera stream and schedule updates.
        self.main_app.cam_stream.start()
        self.initialized = False
        self.event = Clock.schedule_interval(self.update, 1 / 30.0)

    def initialize(self, frame: np.ndarray):
        self.DRAW_MARKERS = True
        img = np.float32(frame) / 255.0
        self.markertracker = MarkerTracker(img)
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        marker_centers = self.markertracker.initial_marker_center
        self.Ox = marker_centers[:, 1]
        self.Oy = marker_centers[:, 0]
        self.nct = len(marker_centers)

        # Convert the first frame to grayscale
        self.old_gray = frame_gray

        # Set the parameters for the Lucas-Kanade method
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )

        # Existing p0 array
        self.p0 = np.array([[self.Ox[0], self.Oy[0]]], np.float32).reshape(-1, 1, 2)
        for i in range(self.nct - 1):
            # New point to be added
            new_point = np.array(
                [[self.Ox[i + 1], self.Oy[i + 1]]], np.float32
            ).reshape(-1, 1, 2)
            # Append new point to p0
            self.p0 = np.append(self.p0, new_point, axis=0)

    def update(self, dt):
        frame = self.main_app.cam_stream.update(dt)
        if frame is None:
            return

        if not self.initialized:
            self.initialize(frame=frame)
            self.initialized = True
        else:
            self.update_marker_view(frame=frame)

            scale = self.zoom_slider.value
            if scale != 1.0:
                frame = rescale(frame, scale=scale)

            add_fps_count_overlay(frame=frame, fps=self.main_app.cam_stream.fps)

            # Add force overlay
            self.add_force_overlay(frame)

            texture = Texture.create(
                size=(frame.shape[1], frame.shape[0]), colorfmt="rgb"
            )
            texture.blit_buffer(frame.tobytes(), colorfmt="rgb", bufferfmt="ubyte")
            texture.flip_vertical()
            self.image_widget.texture = texture

    def add_force_overlay(self, frame):
        """Add force information overlay to the frame"""
        # Check if reference is set
        if self.shear_force_calculator.reference_positions is None:
            status_text = "Press 'Set Reference' or 'R' key to start measuring forces"
            cv2.putText(frame, status_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            return
        
        # Add force magnitude as text overlay
        force_text = f"Shear: {self.current_shear_magnitude:.6f} N"
        cv2.putText(frame, force_text, (10, frame.shape[0] - 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Add component forces
        force_x_text = f"X: {self.current_shear_x:.6f} N"
        force_y_text = f"Y: {self.current_shear_y:.6f} N"
        cv2.putText(frame, force_x_text, (10, frame.shape[0] - 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, force_y_text, (10, frame.shape[0] - 45), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Add force direction arrow if magnitude is significant
        if self.current_shear_magnitude > 0.000001:  # 1μN threshold
            center_x, center_y = frame.shape[1] // 2, frame.shape[0] // 2
            
            # Scale arrow length based on force magnitude (limit max length)
            arrow_scale = min(100, self.current_shear_magnitude * 100000)  # Increased sensitivity
            end_x = int(center_x + self.current_shear_x / (self.current_shear_magnitude + 1e-10) * arrow_scale)
            end_y = int(center_y - self.current_shear_y / (self.current_shear_magnitude + 1e-10) * arrow_scale)  # Flip Y for display
            
            cv2.arrowedLine(frame, (center_x, center_y), (end_x, end_y), 
                           (0, 255, 0), 3, tipLength=0.3)

    def update_marker_view(self, frame: np.ndarray):
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        p1, st, err = cv2.calcOpticalFlowPyrLK(
            self.old_gray, frame_gray, self.p0, None, **self.lk_params
        )

        # Select good points
        good_new = p1[st == 1]
        good_old = self.p0[st == 1]

        if len(good_new) < self.nct:
            log_message(f"All points did not converge")
        else:
            # Update points for next iteration
            self.p0 = good_new.reshape(-1, 1, 2)
            
            # Calculate shear forces
            current_positions = good_new
            self.current_shear_x, self.current_shear_y, self.current_shear_magnitude = \
                self.shear_force_calculator.calculate_shear_force(current_positions)
            
            # Update UI labels with better feedback
            if self.shear_force_calculator.reference_positions is None:
                self.force_x_label.text = "Shear Force X: -- N (Press 'Set Reference' button or 'R' key)"
                self.force_y_label.text = "Shear Force Y: -- N (Press 'Set Reference' button or 'R' key)"
                self.force_mag_label.text = "Magnitude: -- N (Press 'Set Reference' button or 'R' key)"
            else:
                # Show more precision for small forces
                self.force_x_label.text = f"Shear Force X: {self.current_shear_x:.6f} N"
                self.force_y_label.text = f"Shear Force Y: {self.current_shear_y:.6f} N"
                self.force_mag_label.text = f"Magnitude: {self.current_shear_magnitude:.6f} N"
            
            # Log data if recording
            if self.is_logging_data:
                self.shear_force_logger.add_frame(
                    current_positions, 
                    self.current_shear_x, 
                    self.current_shear_y, 
                    self.current_shear_magnitude
                )

        # Draw the tracks and markers
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            ix = int(self.Ox[i])
            iy = int(self.Oy[i])
            
            # Draw displacement arrow
            cv2.arrowedLine(
                frame,
                (ix, iy),
                (int(a), int(b)),
                (255, 255, 255),
                thickness=2,
                line_type=cv2.LINE_8,
                tipLength=0.15,
            )

            if self.DRAW_MARKERS:
                # Draw marker circles
                cv2.circle(frame, (int(a), int(b)), 5, (0, 255, 255), -1)
                cv2.circle(frame, (ix, iy), 3, (255, 0, 0), -1)

        self.old_gray = frame_gray.copy()

    def stop(self):
        if self.event:
            self.event.cancel()
        if self.main_app.cam_stream.camera:
            self.main_app.cam_stream.camera.release()

    def take_screenshot(self, instance=None):
        self.main_app.cam_stream.save_screenshot(filepath=self.data_folder_path)

    def toggle_recording(self, instance=None):
        if not self.is_logging_data:
            self.is_logging_data = True
            self.record_data_btn.text = "Stop Recording"
            log_message("Started recording shear force data")
        else:
            self.shear_force_logger.save_data(folder=self.data_folder_path)
            self.is_logging_data = False
            self.record_data_btn.text = "Start Recording"
            log_message("Stopped recording and saved shear force data")

    def on_key_down(self, window, key, *args):
        # Space key for screenshot
        if key == 32:
            self.take_screenshot()
        # R key for setting reference
        elif key == ord('r') or key == ord('R'):
            self.set_reference()

    def open_screenshot_folder_choice(self, instance):
        popup = FileChooserPopup(self.select_screenshot_folder)
        popup.open()

    def select_screenshot_folder(self, path):
        if path:
            self.data_folder_path = path
            self.data_folder_label.text = f"Target Folder: {self.data_folder_path}"


if __name__ == "__main__":
    import argparse
    from config import GSConfig

    parser = argparse.ArgumentParser(
        description="Run the Gelsight Shear Force Measurement with an optional config file."
    )
    parser.add_argument(
        "--gs-config",
        type=str,
        default=None,
        help="Path to the JSON configuration file. If not provided, default config is used.",
    )

    args = parser.parse_args()

    if args.gs_config is not None:
        log_message(f"Provided config path: {args.gs_config}")
    else:
        log_message(f"Didn't provide custom config path.")
        log_message(
            f"Using default config path './default_config.json' if such file exists."
        )
        log_message(
            f"Using default_config variable from 'config.py' if './default_config.json' is not available"
        )
        args.gs_config = "default_config.json"

    gs_config = GSConfig(args.gs_config)
    GelsightShearForce(config=gs_config.config).run()