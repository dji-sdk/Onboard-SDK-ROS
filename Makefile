SHELL := /bin/bash

default:
	source /opt/ros/melodic/setup.bash && cd catkin_ws && catkin clean -f -i -y && catkin build -j3
	rm -f catkin_ws/install/lib/pkgconfig/catkin_tools_prebuild.pc
	@echo "Built!!!!"

install:
	rm -rf catkin_ws/install/share/catkin_tools_prebuild
	install -d $(DESTDIR)/opt/ros/melodic/
	cp -f -p -r catkin_ws/install/lib $(DESTDIR)/opt/ros/melodic
	cp -f -p -r catkin_ws/install/share $(DESTDIR)/opt/ros/melodic
	cp -f -p -r catkin_ws/install/include $(DESTDIR)/opt/ros/melodic


clean:
	@echo "Cleaning"
	rm -rf catkin_ws
