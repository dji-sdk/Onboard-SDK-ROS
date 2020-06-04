SHELL:= /bin/bash
.PHONY:	configure


build: configure
	ROS_LANG_DISABLE=genlisp:gennodejs:geneus source /opt/ros/melodic/setup.bash && cd catkin_ws && catkin build --no-status

install:
	$(RM) -rf catkin_ws/install/share/{catkin_tools_prebuild,roseus}
	install -d $(DESTDIR)/opt/ros/melodic/
	cp -p -r catkin_ws/install/lib $(DESTDIR)/opt/ros/melodic
	cp -p -r catkin_ws/install/share $(DESTDIR)/opt/ros/melodic
	cp -p -r catkin_ws/install/include $(DESTDIR)/opt/ros/melodic
	rm -f $(DESTDIR)/opt/ros/melodic/lib/pkgconfig/catkin_tools_prebuild.pc

configure:
	mkdir -p catkin_ws/src
	cd catkin_ws && catkin init --workspace . >/dev/null
	cd catkin_ws && catkin config --install
	cp -r dji_sdk dji_sdk_demo catkin_ws/src
	
clean:
	$(RM) -rf catkin_ws 


