import tkinter
import customtkinter
from tkintermapview import TkinterMapView
import cv2

customtkinter.set_appearance_mode("Dark")  # Modes: "System" (standard), "Dark", "Light"
customtkinter.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"


class App(customtkinter.CTk):

    WIDTH = 780
    HEIGHT = 520

    def __init__(self):
        super().__init__()

        self.title("Mission Planner")
        self.geometry(f"{App.WIDTH}x{App.HEIGHT}")
        self.protocol("WM_DELETE_WINDOW", self.on_closing)  # call .on_closing() when app gets closed

        # ============ create two frames ============#

        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        self.frame_left = customtkinter.CTkFrame(master=self,
                                                 width=180)



        self.frame_left.grid(row=0, column=0, sticky="nswe", padx=20, pady=20)

        self.frame_right = customtkinter.CTkFrame(master=self)
        self.frame_right.grid(row=0, column=1, sticky="nswe", padx=20, pady=20)
        


        # creating map_widget on the right frame 
        self.map_widget = TkinterMapView(width = self.frame_right._current_width, height = self.frame_right._current_height, corner_radius=6)
        self.map_widget.grid(row=0, column=1, sticky="nswe", padx=20, pady=20)


        #self.frame_left.grid_rowconfigure(0, minsize=10)   # empty row with minsize as spacing
        #self.frame_left.grid_rowconfigure(5, weight=1)  # empty row as spacing
        #self.frame_left.grid_rowconfigure(8, minsize=20)    # empty row with minsize as spacing
        #self.frame_left.grid_rowconfigure(11, minsize=10)  # empty row with minsize as spacing




        #create useful button on the left plane 

        self.label_1 = customtkinter.CTkLabel(master=self.frame_left,
                                              text="Mission Menu",
                                              text_font=("Roboto Medium", -16))  # font name and size in px
        self.label_1.grid(row=1, column=0, pady=10, padx=10)



        # create button to clear waypoints
        self.button_1 = customtkinter.CTkButton(master=self.frame_left,
                                                text="Clear",
                                                command=self.clear_waypoints)
        self.button_1.grid(row=2, column=0, pady=10, padx=20)


        # create button to upload waypoints
        self.button_2 = customtkinter.CTkButton(master=self.frame_left,
                                                text="Upload",
                                                command=self.upload)
        self.button_2.grid(row=3, column=0, pady=10, padx=20)

        #create button to start the mission
        self.button_3 = customtkinter.CTkButton(master=self.frame_left,
                                                text="Start",
                                                command=self.start)
        self.button_3.grid(row=4, column=0, pady=10, padx=20)


        #create button to stop the mission
        self.button_4 = customtkinter.CTkButton(master=self.frame_left,
                                                text="Stop",
                                                command=self.stop)
        self.button_4.grid(row=5, column=0, pady=10, padx=20)


        #create button to configure the setting of mission
        self.button_5 = customtkinter.CTkButton(master=self.frame_left,
                                                text="Config",
                                                command=self.config)
        self.button_5.grid(row=6, column=0, pady=10, padx=20)



        self.label_2 = customtkinter.CTkLabel(master=self.frame_left,
                                              text="Flight Menu",
                                              text_font=("Roboto Medium", -16))  # font name and size in px
        self.label_2.grid(row=7, column=0, pady=10, padx=10)


        #take off DJI's product
        self.button_6 = customtkinter.CTkButton(master=self.frame_left,
                                                text="Take Off",
                                                command=self.take_off)
        self.button_6.grid(row=8, column=0, pady=10, padx=20)

        #land the DJI's product
        self.button_7 = customtkinter.CTkButton(master=self.frame_left,
                                                text="Land",
                                                command=self.land)
        self.button_7.grid(row=9, column=0, pady=10, padx=20)


       #button to return the robot to its orginal position
        self.button_8 = customtkinter.CTkButton(master=self.frame_left,
                                                text="Go Home",
                                                command=self.go_to_home)
        self.button_8.grid(row=10, column=0, pady=10, padx=20)


        #to add waypoints on the map, by clicking right click mouse
        self.map_widget.add_right_click_menu_command(label="Add Marker",
                                                     command=self.add_marker_event,
                                                     pass_coords=True)

        #list to save the waypoints
        self.marker_list = [] 


        self.label_3 = customtkinter.CTkLabel(master=self.frame_left,
                                              text="Camera",
                                              text_font=("Roboto Medium", -16))  # font name and size in px
        self.label_3.grid(row=11, column=0, pady=10, padx=10)



        #to open another window for Camera
        self.button_9 = customtkinter.CTkButton(master=self.frame_left,
                                                text="Camera",
                                                command=self.open_camera)
        self.button_9.grid(row=12, column=0, pady=10, padx=20)

        self.label_3 = customtkinter.CTkLabel(master=self.frame_left,
                                              text="Map View",
                                              text_font=("Roboto Medium", -16))  # font name and size in px
        self.label_3.grid(row=13, column=0, pady=10, padx=10)


        self.map_option_menu = customtkinter.CTkOptionMenu(self.frame_left, values=["OpenStreetMap", "Google normal", "Google satellite"],
                                                                       command=self.change_map)
        self.map_option_menu.grid(row=14, column=0, pady=10, padx=20)





        





    def add_marker_event(self,coords):
        print("Add marker:", coords)
        new_marker = self.map_widget.set_marker(coords[0], coords[1])
        self.marker_list.append(new_marker)

    def on_closing(self, event=0):
        self.destroy()

    def clear_waypoints(self):
        for marker in self.marker_list:
            self.map_widget.delete(marker)
        print("Clear Waypoints")

    def upload(self):
        print("Mission Successfully Uploaded")

    def start(self):
        print("Mission Successfully Started")

    def stop(self):
        print("Mission Successfully Stopped")

    def config(self):       
        pass
    
    def take_off(self):
        print("Take Off Successfully")

    def land(self):
        print("Landing Successfully")

    def go_to_home(self):
        print("Returning to Home")

    def open_camera(self):
        print("Camera is Opening")
        vid = cv2.VideoCapture(0)
  
        while(True):
            ret, frame = vid.read()
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        vid.release()
        cv2.destroyAllWindows()


    def change_map(self, new_map: str):
        if new_map == "OpenStreetMap":
            self.map_widget.set_tile_server("https://a.tile.openstreetmap.org/{z}/{x}/{y}.png")
        elif new_map == "Google normal":
            self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=m&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)
        elif new_map == "Google satellite":
            self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)
        

    

    



if __name__ == "__main__":
    app = App()
    app.mainloop()