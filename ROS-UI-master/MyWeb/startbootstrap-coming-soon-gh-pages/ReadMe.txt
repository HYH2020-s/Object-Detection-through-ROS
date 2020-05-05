This folder is uset as the font-end web page for our Real-Time object detection project.

How to run this web page:

1. install ROS kinetic 

2. install rosbridge_server

3. open a new terminal: 
                  $ source /opt/ros/kinetic/setup.bash
		  $ roslaunch rosbridge_server rosbridge_websocket.launch
If connect to ROS successfully the terminal should be as follow:

clients total.
2020-04-30 20:05:52+0100 [-] [ERROR] [1588273552.324810]: [Client 24] [id: publish:/selected_type:3] publish: Cannot infer topic type for topic /selected_type as it is not yet advertised
2020-04-30 20:05:52+0100 [-] [INFO] [1588273552.331344]: [Client 24] Subscribed to /listener
2020-04-30 20:05:55+0100 [-] [INFO] [1588273555.775830]: Client disconnected. 0 clients total.
2020-04-30 20:06:01+0100 [-] [INFO] [1588273561.784066]: Client connected.  1 clients total.
2020-04-30 20:06:02+0100 [-] [ERROR] [1588273562.634541]: [Client 25] [id: advertise:/selected_type:1] advertise: Unable to import msg class String.msg from package std_msgs. Caused by 'module' object has no attribute 'String.msg'
2020-04-30 20:06:02+0100 [-] [ERROR] [1588273562.640477]: [Client 25] [id: publish:/selected_type:2] publish: Cannot infer topic type for topic /selected_type as it is not yet advertised
2020-04-30 20:06:04+0100 [-] [ERROR] [1588273564.113497]: [Client 25] [id: publish:/selected_type:3] publish: Cannot infer topic type for topic /selected_type as it is not yet advertised
2020-04-30 20:06:04+0100 [-] [INFO] [1588273564.122323]: [Client 25] Subscribed to /listener
2020-04-30 20:06:08+0100 [-] [INFO] [1588273568.685218]: Client disconnected. 0 clients total.

4. Specify the path and total number of images want to show on showImage.html file 

eg : total number of images: from image0.jpg to image99.jpg
     Path : "./Image"

     var Path = "./Image/image%s.jpg"

     for(var i=0;i<100;i++){
	Img[i]= Path.format(i);
	}  

5. Run html file on web browser
 
