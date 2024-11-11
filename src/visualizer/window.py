import dearpygui.dearpygui as dpg
import numpy as np

class Window:
    def __init__(self, simulation, initial_speed = 1):
        self.simulation = simulation
        self.zoom = 3
        self.offset = (0, 0)
        self.speed = initial_speed
        self.stop_switch = True
        self.is_running = False
        self.initialized = False

        self.is_dragging = False
        self.old_offset = (0, 0)
        self.zoom_speed = 1

        self.setup()
        self.setup_themes()
        self.create_windows()
        self.create_handlers()
        self.resize_windows()

    def setup(self):
        dpg.create_context()
        dpg.create_viewport(title="TrafficSimulator", width=1280, height=720)
        dpg.setup_dearpygui()

    def setup_themes(self):
        with dpg.theme() as global_theme:

            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 5, category=dpg.mvThemeCat_Core)
                dpg.add_theme_style(dpg.mvStyleVar_FrameBorderSize, 1, category=dpg.mvThemeCat_Core)
                dpg.add_theme_style(dpg.mvStyleVar_WindowBorderSize, 0, category=dpg.mvThemeCat_Core)
                # dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, (8, 6), category=dpg.mvThemeCat_Core)
                dpg.add_theme_color(dpg.mvThemeCol_Button, (90, 90, 95))
                dpg.add_theme_color(dpg.mvThemeCol_Header, (0, 91, 140))
            with dpg.theme_component(dpg.mvInputInt):
                dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (90, 90, 95), category=dpg.mvThemeCat_Core)
            #     dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 5, category=dpg.mvThemeCat_Core)

        dpg.bind_theme(global_theme)

        # dpg.show_style_editor()

        with dpg.theme(tag="RunButtonTheme"):
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (5, 150, 18))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (12, 207, 23))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (2, 120, 10))

        with dpg.theme(tag="StopButtonTheme"):
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (150, 5, 18))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (207, 12, 23))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (120, 2, 10))

    def create_windows(self):
        dpg.add_window(
            tag="MainWindow",
            label="Simulation",
            no_close=True,
            no_collapse=True,
            no_resize=True,
            no_move=True
        )
        
        dpg.add_draw_node(tag="OverlayCanvas", parent="MainWindow")
        dpg.add_draw_node(tag="Canvas", parent="MainWindow")

        with dpg.window(
            tag="ControlsWindow",
            label="Controls",
            no_close=True,
            no_collapse=True,
            no_resize=True,
            no_move=True
        ):
            with dpg.collapsing_header(label="Simulation Control", default_open=True):

                with dpg.group(horizontal=True):
                    dpg.add_button(label="Run", tag="RunStopButton", callback=self.toggle)
                    dpg.add_button(label="Next frame", callback=self.simulation.update)

                dpg.add_slider_int(tag="SpeedInput", label="Speed", min_value=1, max_value=100,default_value=self.speed, callback=self.set_speed)
            
            with dpg.collapsing_header(label="Simulation Status", default_open=True):

                with dpg.table(header_row=False):
                    dpg.add_table_column()
                    dpg.add_table_column()
                    
                    with dpg.table_row():
                        dpg.add_text("Status:")
                        dpg.add_text("_", tag="StatusText")

                    with dpg.table_row():
                        dpg.add_text("Time:")
                        dpg.add_text("_s", tag="TimeStatus")

                    with dpg.table_row():
                        dpg.add_text("Frame:")
                        dpg.add_text("_", tag="FrameStatus")

            with dpg.collapsing_header(label="Statistics", default_open=True):
                with dpg.table(header_row=False):
                    dpg.add_table_column()
                    dpg.add_table_column()
                    
                    with dpg.table_row():
                        dpg.add_text("No. of Vehicles:")
                        dpg.add_text("_", tag="NumberOfVeh")
                    
                    with dpg.table_row():
                        dpg.add_text("Average Delay:")
                        dpg.add_text("_", tag="AvgDelay")
                    
                    with dpg.table_row():
                        dpg.add_text("Average Stops:")
                        dpg.add_text("_", tag="AvgStops")
                    
                    with dpg.table_row():
                        dpg.add_text("Average FC:")
                        dpg.add_text("_", tag="AvgFC")
            
            
            with dpg.collapsing_header(label="Camera Control", default_open=True):
    
                dpg.add_slider_float(tag="ZoomSlider", label="Zoom", min_value=0.1, max_value=100, default_value=self.zoom,callback=self.set_offset_zoom)            
                with dpg.group():
                    dpg.add_slider_float(tag="OffsetXSlider", label="X Offset", min_value=-100, max_value=100, default_value=self.offset[0], callback=self.set_offset_zoom)
                    dpg.add_slider_float(tag="OffsetYSlider", label="Y Offset", min_value=-100, max_value=100, default_value=self.offset[1], callback=self.set_offset_zoom)
            
            with dpg.collapsing_header(label="Results", default_open=True):
                with dpg.group(horizontal=False):
                    vehicle_class = None
                    dpg.add_button(label="Generate Trajectory File", callback=lambda: self.simulation.generate_trajectory_file(vehicle_class))
                    dpg.add_button(label="Generate Detector File", callback=lambda: self.simulation.generate_detector_file())
                    dpg.add_button(label="Generate Signal File", callback=lambda: self.simulation.generate_signal_indication_file())
                    dpg.add_button(label="Generate Number of Veh File", callback=lambda: self.simulation.generate_vehicle_no_file())
                    dpg.add_button(label="Generate Queue File", callback=lambda: self.simulation.generate_queue_file())

                    #TODO: add a button to generate summary file
                    # dpg.add_button(label="Generate Summary File", callback=self.simulation.update)

    def resize_windows(self):
        width = dpg.get_viewport_width()
        height = dpg.get_viewport_height()

        dpg.set_item_width("ControlsWindow", 300)
        dpg.set_item_height("ControlsWindow", height-38)
        dpg.set_item_pos("ControlsWindow", (0, 0))

        dpg.set_item_width("MainWindow", width-315)
        dpg.set_item_height("MainWindow", height-38)
        dpg.set_item_pos("MainWindow", (300, 0))

    def create_handlers(self):
        with dpg.handler_registry():
            dpg.add_mouse_down_handler(callback=self.mouse_down)
            dpg.add_mouse_drag_handler(callback=self.mouse_drag)
            dpg.add_mouse_release_handler(callback=self.mouse_release)
            dpg.add_mouse_wheel_handler(callback=self.mouse_wheel)
        dpg.set_viewport_resize_callback(self.resize_windows)

    def update_panels(self):
        # Update status text
        if self.is_running:
            dpg.set_value("StatusText", "Running")
            dpg.configure_item("StatusText", color=(0, 255, 0))
        else:
            dpg.set_value("StatusText", "Stopped")
            dpg.configure_item("StatusText", color=(255, 0, 0))
        
        # Update time and frame text
        dpg.set_value("TimeStatus", f"{self.simulation.t:.2f}s")
        dpg.set_value("FrameStatus", self.simulation.frame_count)
        dpg.set_value("NumberOfVeh", self.simulation.get_total_vehicle_count())

        # try: dpg.set_value("AvgDelay", f"{self.simulation.total_delay / self.simulation.get_total_vehicle_count():.3f}s")
        # dpg.set_value("AvgDelay", f"{self.simulation.get_average_delay():.3f}s")
        try: dpg.set_value("AvgDelay", f"{self.simulation.get_average_delay():.3f}s")
        except: dpg.set_value("AvgDelay", "_")

        # try: dpg.set_value("AvgStops", f"{self.simulation.total_stops / self.simulation.get_total_vehicle_count():.3f}")
        try: dpg.set_value("AvgStops", f"{self.simulation.get_average_stops():.3f}")
        except: dpg.set_value("AvgStops", "_")

        # try: dpg.set_value("AvgFC", f"{self.simulation.total_fc / self.simulation.get_total_vehicle_count():.3f}L")
        try: dpg.set_value("AvgFC", f"{self.simulation.get_average_fuel():.3f}mL")
        except: dpg.set_value("AvgFC", "_")

    def mouse_down(self):
        if not self.is_dragging:
            if dpg.is_item_hovered("MainWindow"):
                self.is_dragging = True
                self.old_offset = self.offset
        
    def mouse_drag(self, sender, app_data):
        if self.is_dragging:
            self.offset = (
                self.old_offset[0] + app_data[1]/self.zoom,
                self.old_offset[1] + app_data[2]/self.zoom
            )

    def mouse_release(self):
        self.is_dragging = False

    def mouse_wheel(self, sender, app_data):
        if dpg.is_item_hovered("MainWindow"):
            self.zoom_speed = 1 + 0.01*app_data

    def update_inertial_zoom(self, clip=0.005):
        if self.zoom_speed != 1:
            self.zoom *= self.zoom_speed
            self.zoom_speed = 1 + (self.zoom_speed - 1) / 1.05
        if abs(self.zoom_speed - 1) < clip:
            self.zoom_speed = 1

    def update_offset_zoom_slider(self):
        dpg.set_value("ZoomSlider", self.zoom)
        dpg.set_value("OffsetXSlider", self.offset[0])
        dpg.set_value("OffsetYSlider", self.offset[1])

    def set_offset_zoom(self):
        self.zoom = dpg.get_value("ZoomSlider")
        self.offset = (dpg.get_value("OffsetXSlider"), dpg.get_value("OffsetYSlider"))

    def set_speed(self):
        self.speed = dpg.get_value("SpeedInput")


    def to_screen(self, x, y):
        return (
            self.canvas_width/2 + (x + self.offset[0] ) * self.zoom,
            self.canvas_height/2 + (y + self.offset[1]) * self.zoom
        )

    def to_world(self, x, y):
        return (
            (x - self.canvas_width/2) / self.zoom - self.offset[0],
            (y - self.canvas_height/2) / self.zoom - self.offset[1] 
        )
    
    @property
    def canvas_width(self):
        return dpg.get_item_width("MainWindow")

    @property
    def canvas_height(self):
        return dpg.get_item_height("MainWindow")


    def draw_bg(self, show, color=(250, 250, 250)):
        dpg.draw_rectangle(
            (-10, -10),
            (self.canvas_width+10, self.canvas_height+10), 
            thickness=0,
            fill=color,
            parent="OverlayCanvas",
            show=show
        )

    def draw_axes(self, show, opacity=80):
        x_center, y_center = self.to_screen(0, 0)
        
        dpg.draw_line(
            (-10, y_center),
            (self.canvas_width+10, y_center),
            thickness=2, 
            color=(0, 0, 0, opacity),
            parent="OverlayCanvas",
            show=show
        )
        dpg.draw_line(
            (x_center, -10),
            (x_center, self.canvas_height+10),
            thickness=2,
            color=(0, 0, 0, opacity),
            parent="OverlayCanvas",
            show=show
        )

    def draw_grid(self, show, unit=10, opacity=50):
        x_start, y_start = self.to_world(0, 0)
        x_end, y_end = self.to_world(self.canvas_width, self.canvas_height)

        n_x = int(x_start / unit)
        n_y = int(y_start / unit)
        m_x = int(x_end / unit)+1
        m_y = int(y_end / unit)+1

        for i in range(n_x, m_x):
            dpg.draw_line(
                self.to_screen(unit*i, y_start - 10/self.zoom),
                self.to_screen(unit*i, y_end + 10/self.zoom),
                thickness=1,
                color=(0, 0, 0, opacity),
                parent="OverlayCanvas",
            show=show
            )

        for i in range(n_y, m_y):
            dpg.draw_line(
                self.to_screen(x_start - 10/self.zoom, unit*i),
                self.to_screen(x_end + 10/self.zoom, unit*i),
                thickness=1,
                color=(0, 0, 0, opacity),
                parent="OverlayCanvas",
            show=show
            )

    def draw_segments(self, show):
        for segment in self.simulation.segments[::-1]:
            # (180, 180, 220)
            dpg.draw_polyline(segment.points, color=(164, 164, 166), thickness=segment.link_width*self.zoom, parent="Canvas", show=show)
            # dpg.draw_arrow(segment.points[-1], segment.points[-2], thickness=0, size=2, color=(0, 0, 0, 50), parent="Canvas")

            
            

            
            if segment.has_marking:

                #solid white and yellow edge lines
                dx = segment.points[0][0] - segment.points[1][0]
                dy = segment.points[0][1] - segment.points[1][1]
                angle = np.arctan2(dy, dx)
                dpg.draw_line((segment.points[0][0] + (segment.link_width / 2 - 0.1)* np.sin(angle), segment.points[0][1] - (segment.link_width / 2 - 0.1)* np.cos(angle)),
                            (segment.points[1][0] + (segment.link_width / 2 - 0.1)* np.sin(angle), segment.points[1][1] - (segment.link_width / 2 - 0.1)* np.cos(angle)),
                            thickness=0.1*self.zoom, color=(255,255,255),parent="Canvas",show=show)
                dpg.draw_line((segment.points[0][0] - (segment.link_width / 2 - 0.1)* np.sin(angle), segment.points[0][1] + (segment.link_width / 2 - 0.1)* np.cos(angle)),
                            (segment.points[1][0] - (segment.link_width / 2 - 0.1)* np.sin(angle), segment.points[1][1] + (segment.link_width / 2 - 0.1)* np.cos(angle)),
                            thickness=0.1*self.zoom, color=(247,181,0),parent="Canvas",show=show)
                

                # Stop bars
                if segment.has_traffic_signal:
                    dpg.draw_line((segment.points[1][0] + (segment.link_width / 2 - 0.15)* np.sin(angle), segment.points[1][1] - (segment.link_width / 2 - 0.15)* np.cos(angle)),
                                (segment.points[1][0] - (segment.link_width / 2 - 0.15)* np.sin(angle), segment.points[1][1] + (segment.link_width / 2 - 0.15)* np.cos(angle)),
                                thickness=0.5*self.zoom,color=(255,255,255),parent="Canvas",show=show)
                    
                
                # Passing Marking
                if segment.number_of_lanes > 1:
                    # for i in np.arange(0, segment.length, 12):
                    #     dx = segment.points[1][0] - segment.points[0][0]
                    #     dy = segment.points[1][1] - segment.points[0][1]
                    #     angle =np.degrees(np.arctan2(dy, dx))

                    #     # print(angle, i* np.sin(np.radians(angle)), i* np.cos(np.radians(angle)))
                    #     x_i = round(segment.points[0][0] + i* np.cos(np.radians(angle)), 2)
                    #     y_i = round(segment.points[0][1] + i* np.sin(np.radians(angle)), 2)
                        

                    #     dpg.draw_line(
                    #     (x_i, y_i),
                    #     (x_i-3*np.cos(np.radians(angle)), y_i-3*np.sin(np.radians(angle))),
                    #     thickness=0.1*self.zoom, 
                    #     color=(255,255,255),
                    #     parent="Canvas")

                    # new way of drawing passing marking
                    node = dpg.add_draw_node(parent="Canvas")
                    for i in np.arange(0, segment.length, 12):
                        for lane in segment.lanes:
                            if lane.order == 0:
                                continue
                            lateral_shift = segment.link_width/2 - lane.order * segment.lane_width

                            


                            x_i1 = -i
                            x_i2 = -i - 3

                            dpg.draw_line((x_i1, lateral_shift), (x_i2, lateral_shift),thickness=0.1*self.zoom,color=(255,255,255),parent=node,show=show)
                            

                            position = segment.get_point(1)
                            heading = segment.get_heading(1)
                            translate = dpg.create_translation_matrix(position)
                            rotate = dpg.create_rotation_matrix(heading, [0, 0, 1])
                            dpg.apply_transform(node, translate*rotate)



    def draw_vehicles(self, show):
        for segment in self.simulation.segments:
            for lane in segment.lanes:
                for vehicle in lane.vehicles:
                    # vehicle = self.simulation.vehicles[vehicle_id]
                    progress = vehicle.x / segment.length
                    # print(self.simulation.t, vehicle_id, vehicle.x , segment.length, progress, vehicle.v, vehicle.a)
                    position = segment.get_point(progress)
                    heading = segment.get_heading(progress)

                    node = dpg.add_draw_node(parent="Canvas")


                    lateral_shift = 0 - vehicle.current_seg.link_width/2 + (vehicle.current_lane.order + 1) * vehicle.current_seg.lane_width/2 + vehicle.current_lane.order * vehicle.current_seg.lane_width / 2
                    # print(vehicle.current_seg.id, vehicle.current_lane.order, (0, lateral_shift), (vehicle.l, lateral_shift))

                    if vehicle.veh_class == 1: color = (0, 0, 0)
                    else: color = (0, 0, 255)

                    dpg.draw_line(
                        (-vehicle.l, lateral_shift),
                        (0, lateral_shift),
                        thickness=1.76*self.zoom,
                        color=color,
                        parent=node,show=show
                    )
                    

                    translate = dpg.create_translation_matrix(position)
                    rotate = dpg.create_rotation_matrix(heading, [0, 0, 1])
                    dpg.apply_transform(node, translate*rotate)


    def draw_signals(self, show):
        for signal in self.simulation.traffic_signals:
            for i in range(len(signal.lanes)):
                

                # color = (0, 255, 0) if lane.traffic_signal_state == 'green' else (255, 0, 0)
                # color = (0, 255, 0) if signal.current_cycle[i] else (255, 0, 0)

                for lane in signal.lanes[i]:
                    
                    if lane.traffic_signal_state == 'green': color = (0, 255, 0)
                    elif lane.traffic_signal_state == 'yellow': color = (255, 255, 0)
                    elif lane.traffic_signal_state == 'red': color = (255, 0, 0)
                    # a = 0
                    # position = (
                    #     (1-a)*segment.end[0] + a*segment.start[0],        
                    #     (1-a)*segment.end[1] + a*segment.start[1]
                    # )
                    # self.rotated_box(
                    #     position,
                    #     (1, 3),
                    #     cos=segment.angle_cos, sin=segment.angle_sin,
                    #     color=color)
                    
                    node = dpg.add_draw_node(parent="Canvas")

                    dx = lane.points[0][0] - lane.points[1][0]
                    dy = lane.points[0][1] - lane.points[1][1]
                    angle = np.arctan2(dy, dx)

                    if lane.points[0][0] == lane.points[-1][0]:
                        #if vertical
                        coord1 = (lane.points[-1][0] - lane.segment.lane_width/2, lane.points[-1][1] - 1 * np.sin(angle))
                        coord2 = (lane.points[-1][0] + lane.segment.lane_width/2, lane.points[-1][1] - 1 * np.sin(angle))
                    else:
                        #if horizontal
                        coord1 = (lane.points[-1][0] - 1 * np.cos(angle), lane.points[-1][1] - lane.segment.lane_width/2)
                        coord2 = (lane.points[-1][0] - 1 * np.cos(angle), lane.points[-1][1] + lane.segment.lane_width/2)

                    dpg.draw_line(
                        coord1,          
                        coord2,
                        thickness=0.5*self.zoom,
                        color=color,
                        parent=node,show=show
                    )

                    

    

    def draw_detectors(self, show):
        for detector in self.simulation.detectors.values():
            if detector.detector_call == True: color = (0, 0, 255)
            else: color = (0, 0, 0)
            node = dpg.add_draw_node(parent="Canvas")
            dpg.draw_rectangle(detector.coord1, detector.coord2, color = color, parent=node,show=show)
            # draw_rectangle(pmin, pmax, label=label, user_data=user_data, use_internal_label=use_internal_label, tag=tag, parent=parent, before=before, show=show, color=color, color_upper_left=color_upper_left, color_upper_right=color_upper_right, color_bottom_right=color_bottom_right, color_bottom_left=color_bottom_left, fill=fill, multicolor=multicolor, rounding=rounding, thickness=thickness, corner_colors=corner_colors, **kwargs)


    def apply_transformation(self):
        screen_center = dpg.create_translation_matrix([self.canvas_width/2, self.canvas_height/2, -0.01])
        translate = dpg.create_translation_matrix(self.offset)
        scale = dpg.create_scale_matrix([self.zoom, self.zoom])
        dpg.apply_transform("Canvas", screen_center*scale*translate)


    def render_loop(self, show):
        # if self.simulation.t + (1/60) * self.speed  > self.simulation.duration:
        # if self.simulation.frame_count + 60 > self.simulation.duration * 60:

        if abs((self.simulation.frame_count) - (self.simulation.duration * (1/self.simulation.dt))) < abs((self.simulation.frame_count + (1/self.simulation.dt)) - (self.simulation.duration * (1/self.simulation.dt))) and self.stop_switch == True:
            self.stop_switch = False
            # self.simulation.generate_trajectory_file()
            # self.simulation.generate_detector_file()
            self.stop()
            # close window
            if self.simulation.destroy: dpg.stop_dearpygui()
        
        # Events
        self.update_inertial_zoom()
        self.update_offset_zoom_slider()

        # Remove old drawings
        dpg.delete_item("OverlayCanvas", children_only=True)
        dpg.delete_item("Canvas", children_only=True)
        # New drawings
        if show:
            self.draw_bg(show)
            self.draw_axes(show)
            self.draw_grid(show, unit=10)
            self.draw_grid(show, unit=50)
            self.draw_segments(show)
            self.draw_vehicles(show)
            self.draw_signals(show)
            self.draw_detectors(show)

            # Apply transformations
            self.apply_transformation()

        # Update panels
        self.update_panels()

        # Update simulation
        if self.is_running:
            self.simulation.run(self.speed)

    def show(self, show=True):
        dpg.show_viewport()
        if self.is_running or (not self.is_running and self.initialized):
            while dpg.is_dearpygui_running():
                self.render_loop(show)
                dpg.render_dearpygui_frame()
        else:
            while dpg.is_dearpygui_running():
                dpg.render_dearpygui_frame()
        dpg.destroy_context()

    def run(self):
        self.is_running = True
        dpg.set_item_label("RunStopButton", "Stop")
        dpg.bind_item_theme("RunStopButton", "StopButtonTheme")

    def stop(self):
        self.is_running = False
        dpg.set_item_label("RunStopButton", "Run")
        dpg.bind_item_theme("RunStopButton", "RunButtonTheme")
        if not self.simulation.destroy: 
            print('\nSimulation Results at time {:.1f}:'.format(self.simulation.t))
            print("No. of Vehicles", self.simulation.get_total_vehicle_count())
            print("AvgDelay", f"{self.simulation.get_average_delay():.3f}s")
            print("AvgStops", f"{self.simulation.get_average_stops():.3f}")
            print("AvgFC", f"{self.simulation.get_average_fuel():.3f}mL")
    
    def initialize(self, is_running=False):
        self.simulation.initialize()
        self.is_running = is_running
        self.initialized = True

        if self.is_running:
            dpg.set_item_label("RunStopButton", "Stop")
            dpg.bind_item_theme("RunStopButton", "StopButtonTheme")
        else:
            dpg.set_item_label("RunStopButton", "Run")
            dpg.bind_item_theme("RunStopButton", "RunButtonTheme")
            
        

    def toggle(self):
        if self.is_running: self.stop()
        else: self.run()
    
    def generate_output(self):
        #tobe updated with commands. current implementation is just a placeholder.
        if self.is_running: self.stop()
        else: self.run()