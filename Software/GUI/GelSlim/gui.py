from __future__ import print_function
import tkSimpleDialog as simpledialog
from Tkinter import *
import Tkinter , Tkconstants, tkFileDialog
import tkMessageBox as messagebox
import os, signal, rosgraph, time, roswtf
from os import path
import socket
import sys
import fileinput
import argparse
from subprocess import Popen, PIPE


root_calib= None
root_recon= None
filename = None
tablename = None
BallDiam = str(4.76)
Pixdiam = str(80)
local_ip = None
raspi_ip = None
res_w=str(427)
res_h=str(320)
ip_count=0
ball_count=0
funny=0

ap = argparse.ArgumentParser()
ap.add_argument("-a","--automatic",    required=False, type=int, help="[1] find IP addresses automatically, [2] Manually input IP addresses.")
args=vars(ap.parse_args())



#CONNECT TO RASPBERRY PI (FINISHED)
def Connect():
	global funny
	if rosgraph.is_master_online(): 
		print('[INFO] Raspberry Pi is already connected...')
		funny += 1
		if funny ==10 :
			print ("HA HA verrry funny...")
		if funny ==13:
			print ("why")
		if funny ==14:
			print ("are")
		if funny ==15:
			print ("you")
		if funny ==16:
			print ("so")
		if funny ==17:
			print ("disrespectful")
		if funny ==18:
			print ("?")
		if funny ==20:
			print ("Goodbye")
			on_closing()
		if funny ==25:
			print (":P")
		if funny ==30:
			os.system("rosnode kill --all")
			pid=os.fork()
			os.kill(pid,signal.SIGSTOP)
			root.destroy()
			os.system("pkill roscore")
			os.system("pkill ssh")
			os.system("pkill -n bash")
			os.system("pkill bash")
			exit()	
	else:
		unconnected()
	#os.system("./camera.sh")


#DISPLAY CAMERA FEED (FINISHED)
def CameraFeed():
	if rosgraph.is_master_online(): 
		os.system("./camera.sh")
	else:
		unconnected()
		os.system("./camera.sh")


#SENSOR CALIBRATION
def Calibration():
	global root_calib
	try:
		root_calib.destroy()
		Calibration()
	except:
		root_calib = Tk()
		root_calib.title("GelSlim - Sensor Calibration")
		root_calib.geometry("500x250")
		W=59; H=2; d=50; 
		y1=0; y2=y1+d; y3=y2+d; y4=y3+d; y5=y4+d;
		B1 = Button(root_calib, text="Reconfigure Camera Parameters",width=W, height=H, command =Reconfigure).place(x=0,y=y1)
		B2 = Button(root_calib, text="Capture New Calibration Data",width=W, height=H, command =capture_data).place(x=0,y=y2)
		B3 = Button(root_calib, text="Label Calibration Data",width=W, height=H, command =label_data).place(x=0,y=y3)
		B4 = Button(root_calib, text="Load Existing Calibration Table",width=W, height=H, command =load_data).place(x=0,y=y4)
		B5 = Button(root_calib, text="Change Calibration Ball Parameters",width=W, height=H, command =ball_param).place(x=0,y=y5)
def Reconfigure():
	if rosgraph.is_master_online(): 
		print("[INFO] Opening rosqt_reconfigure...")
		os.system("./reconfigure.sh")
		os.system("./camera.sh")
	else:
		unconnected()
		print("[INFO] Opening rosqt_reconfigure...")
		os.system("./reconfigure.sh")
		os.system("./camera.sh")
def capture_data():
	global image_num
	global root_calib
	if rosgraph.is_master_online(): 
		print('[INFO] Beginning data capture...')
		image_num = simpledialog.askstring('Calibration Image Count','Please input an integer valued number of calibration images to capture.')
		root_calib.destroy()
		os.system("python img_processor.py -i "+ image_num)
	else:
		print("[INFO] Connecting to Raspberry Pi...")
		os.system("./ROS.sh")
		print("[INFO] Connected to Raspberry Pi...")
		print('[INFO] Beginning data capture...')
		image_num = simpledialog.askstring('Calibration Image Count','Please input an integer valued number of calibration images to capture.')
		root_calib.destroy()
		os.system("python img_processor.py -i "+ image_num)
		#print("[INFO] Press Ctrl+C to exit")
		#Dialog box asking how many calibration photos to take
		#change calibration.py to require "enter" to move forward
def label_data():
	global BallDiam
	global Pixdiam
	global root_calib
	root_calib.destroy()
	os.system("python calibration.py -d "+BallDiam+ " -p "+Pixdiam)
def load_data():
	global tablename
	global root_calib
	global root_recon
	tablename = tkFileDialog.askopenfilename(initialdir = "./load",parent=root, title='Calibration File',filetypes=[("Table","*.npy")])
	if tablename:
		print("[INFO] Loaded Calibration Table...")
		if root_calib != None:
			root_calib.destroy()
	else:
		if root_calib != None:
			root_calib.destroy()
		else:
			root_recon.destroy()
def ball_param():
	global root_calib
	global BallDiam
	global Pixdiam
	global ball_count
	ball_count+=1
	print('[INFO] Updating calibration ball parameters...')
	print('[INFO] To change default parameters edit gui.py lines 16-17')
	print('[INFO] Printing Current Parameters...')
	print('[INFO] Calibration Ball Diameter: '+str(BallDiam)+' mm')
	print('[INFO] Calibration Ball Diameter: '+str(Pixdiam)+' px')
	if ball_count<=1:
		BallDiam_new=simpledialog.askstring('self.BallDiam','Please input the diameter of the calibration ball in millimeters')
		if BallDiam_new == '' or BallDiam_new ==None:
			print('[INFO] Value update cancelled')
			print('[INFO] Calibration Ball Diameter: '+str(BallDiam)+' mm')
		else:
			BallDiam=BallDiam_new
			print('[INFO] Value update cancelled')
			print('[INFO] Calibration Ball Diameter: '+str(BallDiam)+' mm')
		Pixdiam_new=simpledialog.askstring('self.Pixmm','Please input the diameter of the calibration ball in pixels')
		if Pixdiam_new =='' or Pixdiam_new ==None:
			print('[INFO] Value update cancelled')
			print('[INFO] Calibration Ball Diameter: '+str(Pixdiam)+' px')
		else:
			Pixdiam=Pixdiam_new
			print('[INFO] Value update complete')
			print('[INFO] Calibration Ball Diameter: '+str(Pixdiam)+' px')
		print('[INFO] Parameter update complete...')
		try:
			ball_count=0
			root_calib.destroy()
		except:
			ball_count=0
			print('[INFO] Closing dialog windows...')
	else:
		print('[INFO] A dialog window is already open...')

#3D RECONSTRUCTION
def Reconstruction():
	global root_recon
	try:
		root_recon.destroy()
		Reconstruction()
	except:
		root_recon = Tk()
		root_recon.title("GelSlim - 3D Reconstruction")
		root_recon.geometry("500x150")
		W=59; H=2; d=50; 
		y1=0; y2=y1+d; y3=y2+d; y4=y3+d
		B1 = Button(root_recon, text="Capture New Reconstruction Images",width=W, height=H, command =recon_cap).place(x=0,y=y1)
		B2 = Button(root_recon, text="Select a file to reconstruct",width=W, height=H, command =file_select).place(x=0,y=y2)
		B3 = Button(root_recon, text ="Realtime Depth Plot", width=W, height=H, command=realtime).place(x=0, y=y3)
def recon_cap():
	global root_recon
	if rosgraph.is_master_online(): 
		print('[INFO] Beginning data capture...')
		root_recon.destroy()
		image_num = simpledialog.askstring('Reconstruction Image Count','Please input an integer valued number of reconstruction images to capture.')
		os.system("python recon_jpg.py -i "+ image_num)
	else:
		print("[INFO] Connecting to Raspberry Pi...")
		os.system("./ROS.sh")
		print("[INFO] Connected to Raspberry Pi...")
		print('[INFO] Beginning data capture...')
		root_recon.destroy()
		image_num = simpledialog.askstring('Reconstruction Image Count','Please input an integer valued number of reconstruction images to capture.')
		os.system("python recon_jpg.py -i "+ image_num)
def file_select():
	global root_recon
	global filename
	global tablename
	if tablename !=None:
		filename = tkFileDialog.askopenfilename(initialdir = "./reconstruction/images/",parent=root,title='3D Reconstruction Image',filetypes=[("JPG Image","*.jpg")])
		if filename:
			print("[INFO] Plotting 3D Reconstruction...")
			exe_str=str("python test_poisson.py -n "+filename +" -c"+tablename +" -d "+BallDiam +" -p "+Pixdiam)
			root_recon.destroy()
			os.system(exe_str)
		else:
			root_recon.destroy()
	else:
		print("[INFO] Calibration Table is not loaded...")
		if path.exists("./load/table_3.npy"):
			tablename=("./load/table_3.npy")
			print("[INFO] Default Loading table_3.npy...")
			file_select()
		elif path.exists("./load/table_3_smooth.npy"):
			tablename =("./load/table_3_smooth.npy")
			print("[INFO] Default Loading table_3_smooth.npy...")
			file_select()
		else:
			load_data()
			file_select()
def realtime():
	global tablename
	global root_recon
	if tablename != None:
		if rosgraph.is_master_online(): 
			root_recon.destroy()
			os.system("python depth_realtime.py -c "+tablename)
		else:
			print("[INFO] Connecting to Raspberry Pi...")
			os.system("./ROS.sh")
			print("[INFO] Connected to Raspberry Pi...")
			print("[INFO] Opening Realtime 3D Reconstruction...")
			os.system("python depth_realtime.py -c" +tablename)
	else:
		print("[INFO] Calibration Table is not loaded...")
		if path.exists("./load/table_3.npy"):
			tablename=("./load/table_3.npy")
			print("[INFO] Default Loading table_3.npy...")
			root_recon.destroy()
			realtime()
		elif path.exists("./load/table_3_smooth.npy"):
			tablename =("./load/table_3_smooth.npy")
			print("[INFO] Default Loading table_3_smooth.npy...")
			root_recon.destroy()
			realtime()
		else:
			load_data()
			root_recon.destroy()
			realtime()


#INCIPIENT SLIP (FINISHED)
def SlipDetector():
	if rosgraph.is_master_online(): 
		print("[INFO] Opening Slip Detector...")
		print("[INFO] Press Ctrl+C to exit")
		os.system("python slip_detector_both.py")
	else:
		print("[INFO] Connecting to Raspberry Pi...")
		os.system("./ROS.sh")
		print("[INFO] Connected to Raspberry Pi...")
		print("[INFO] Opening Slip Detector...")
		print("[INFO] Press Ctrl+C to exit")
		os.system("python slip_detector_both.py")


#HELPER FUNCTIONS
def unconnected():
	global local_ip
	global raspi_ip
	if local_ip != None and raspi_ip !=None:
		print('[INFO] Local IP Address : '+local_ip)
		print('[INFO] Raspberry Pi IP Address : ' +raspi_ip)
		print("[INFO] Setting Raspberry Pi IP Address...")
		file_data=('#!/bin/bash\ncd\nssh -tt raspi@'+raspi_ip+' << EOF\nroslaunch raspberry_new.launch\nEOF')
		read_obj =open('./raspi_connect.sh','w')
		read_obj.writelines(file_data)
		read_obj.close()
		
		read_obj=open('./reboot.sh','r')
		file_data_reboot=read_obj.readlines()
		file_data_reboot[4]=(('ssh -tt raspi@'+raspi_ip+' << EOF\n'))
		read_obj =open('./reboot.sh','w')
		read_obj.writelines(file_data_reboot)
		read_obj.close()
		print("[INFO] Connecting to Raspberry Pi...")
		launch_write()
		env_write()
		os.system("./ROS.sh")
		print("[INFO] Verifying Connection...")
		error_handler()
	elif local_ip != None and raspi_ip== None:
			print("[INFO] No Raspberry Pi Detected...")
			print("[INFO] Please try again or exit the program")
	elif local_ip == None and raspi_ip == None:
		host_name=socket.gethostname()
		local_ip=socket.gethostbyname(host_name)
		os.system(' nmap -sP 192.168.1.0/24')
		open('./load/arp_cache.txt','w').close()
		os.system("arp -a >>./load/arp_cache.txt")
		print("[INFO] Searching for Raspberry Pi...")
		with open('./load/arp_cache.txt','r') as read_obj:
			for line in read_obj:
				if 'raspi' in line:
					print("\n[INFO] Raspberyy Pi detected...")
					s=line
					raspi_ip=s[s.find('(')+1:s.find(')')]
					unconnected()
def on_closing():
	if messagebox.askokcancel("Quit", "Do you want to quit?"):
		if rosgraph.is_master_online(): 
			os.system("rosnode kill --all")
			pid=os.fork()
			os.kill(pid,signal.SIGSTOP)
			root.destroy()
			os.system("pkill roscore")
			os.system("pkill ssh")
			os.system("pkill -n bash")
			os.system("pkill bash")
			exit()
		else:
			pid=os.fork()
			os.kill(pid,signal.SIGSTOP)
			root.destroy()
			os.system("pkill roscore")
			os.system("pkill ssh")
			os.system("pkill -n bash")
			os.system("pkill bash")
			exit()
def flag():
	flag_auto='{}'.format(args["automatic"])
	global local_ip
	global raspi_ip
	#print (flag_auto)
	if flag_auto == '1':
		local_ip = None
		raspi_ip = None
	elif flag_auto =='2':
		local_ip = simpledialog.askstring('Local IP Address','Please input the Local IP Address')
		raspi_ip = simpledialog.askstring('Local IP Address','Please input the Raspberry Pi IP Address')
	else:	
		local_ip = '192.168.1.83'
		raspi_ip = '192.168.1.219'
		print('[INFO] Default Local IP Address: '+local_ip)
		print('[INFO] Default Raspberry Pi Address: '+raspi_ip)
		print('[INFO] Change default IP Addresses in gui.py line 21-22')
		print('[INFO] gui.py -a 1 [Finds IP Addresses automatically]')
		print('[INFO] gui.py -a 2 [Set IP Addresses manually]')
		print('[INFO] gui.py -h   [Help]')
def error_handler():
	global ip_count
	try:
		time.sleep(10)
		master=rosgraph.Master("")
		master.lookupNode('raspicam_node1')
		print("[INFO] raspicam_node1 exists...")
		stdout = Popen('roswtf', shell=True, stdout=PIPE).stdout
		output = stdout.read()
		verify=(output.split("\n")[-2])
		sys.stdout.flush()
		if verify !='No errors or warnings':
			print("================================================================================\n")
			print("[INFO] ERROR: unable to communicate with raspicam_node1")
			raise Exception()
		else:
			print('[INFO] Node connection verified...')
			print('[INFO] Connected to Raspberry Pi...')
	except: 
		print('[INFO] ERROR: unable to establish connection with Raspberry Pi...')
		print('[INFO] ERROR: connection timeout exception...')
		print('[INFO] Please see troubleshooting guide for further assistance...')
		print('[INFO] Forcing roscore Exit...')
		if ip_count ==3:
			print('[INFO] ERROR: unable to establish connection with Raspberry Pi...')
			print('[INFO] ERROR: connection timeout exception...')
			print('[INFO] Please see troubleshooting guide for further assistance...')
			print('[INFO] Forcing roscore Exit...')
			print('[INFO] ERROR: Boot Loop Exception...')
			disconnect()
		if rosgraph.is_master_online(): 
			ip_count+=1
			os.system("rosnode kill --all")
			pid=os.fork()
			os.kill(pid,signal.SIGSTOP)
			os.system("pkill roscore")
			os.system("pkill ssh")
			os.system("pkill -n bash")
			os.system("pkill bash")
			print("[INFO] Rebooting Raspberry Pi and re-trying connection...")
			print("[INFO] Waiting for reboot, this may take a minute...")
			os.system('./reboot.sh')
			for i in xrange(200,0,-1):
				line=('[INFO] Reboot Time Left: '+str(i)+' seconds   ')
				print(line, end='\r')
				time.sleep(1)
				sys.stdout.flush()
			sys.stdout.flush()
			unconnected()
		else:
			pid=os.fork()
			os.kill(pid,signal.SIGSTOP)
			os.system("pkill roscore")
			os.system("pkill ssh")
			os.system("pkill -n bash")
			os.system("pkill bash")
def launch_write():
	global res_w
	global res_h
	print('[INFO] Writing updated .launch file to Raspberry Pi...')
	read_obj=open('./load/raspberry_new.launch','r')
	file_data=read_obj.readlines()
	file_data[3]= '<machine name="raspberry_machine" address="'+raspi_ip+'" user="raspi" password="raspi" timeout="5.0" env-loader="/home/raspi/suction_env.sh" />\n'
	file_data[23]=('    <param name="width" value="'+res_w+'"/>\n')
	file_data[24]=('    <param name="height" value="'+res_h+'"/>\n')
	print('[INFO] Camera Resolution set to '+res_w+'x'+res_h)
	#print(*file_data)
	read_obj =open('./load/raspberry_new.launch','w')
	read_obj.writelines(file_data)
	read_obj.close()
	command_str="scp ./load/raspberry_new.launch raspi@"+raspi_ip+":/home/raspi/"
	os.system(command_str)
	#os.system("exit")
def env_write():
	global local_ip
	global raspi_ip
	print('[INFO] Writing updated environment file to Raspberry Pi...')
	read_obj=open('./load/suction_env.sh','r')
	file_data=read_obj.readlines()
	file_data[3]=("export ROS_MASTER_URI=http://"+local_ip+":11311/\n")
	file_data[4]=("export ROS_IP="+raspi_ip+"\n")
	file_data[5]=("export ROS_HOSTNAME="+raspi_ip+"\n")
	#print(*file_data)
	read_obj =open('./load/suction_env.sh','w')
	read_obj.writelines(file_data)
	read_obj.close()
	command_str=("scp ./load/suction_env.sh raspi@"+raspi_ip+":/home/raspi/")
	os.system(command_str)
	#os.system("exit")
def disconnect():
	print('[INFO] Disconnecting from Raspberry Pi...')
	print('[INFO] Forcing roscore Exit...')
	if rosgraph.is_master_online(): 
		os.system("rosnode kill --all")
		pid=os.fork()
		os.kill(pid,signal.SIGSTOP)
		os.system("pkill roscore")
		os.system("pkill ssh")
		os.system("pkill -n bash")
		os.system("pkill bash")
	else:
		pid=os.fork()
		os.kill(pid,signal.SIGSTOP)
		os.system("pkill roscore")
		os.system("pkill ssh")
		os.system("pkill -n bash")
		os.system("pkill bash")

#MAIN
if __name__ == '__main__':
    
    root = Tk()
    root.title("GelSlim")
    root.geometry("500x300")
    W=59; H=2; d=50; 
    y1=0; y2=y1+d; y3=y2+d; y4=y3+d; y5=y4+d; y6=y5+d; 
    B1 = Button(root, text="Connect to Raspberry Pi",width=W, height=H, command =Connect).place(x=0,y=y1)
    B1 = Button(root, text="Display Camera Feed",width=W, height=H, command =CameraFeed).place(x=0,y=y2)
    B2 = Button(root, text ="Sensor Calibration", width=W, height=H, command=Calibration).place(x=0, y=y3)
    B3 = Button(root, text ="3D Reconstruction", width=W, height=H, command=Reconstruction).place(x=0, y=y4)
    B4 = Button(root, text ="Incipient Slip", width=W, height=H, command=SlipDetector).place(x=0, y=y5)
    B5 = Button(root, text ="Disconnect from Raspberry Pi", width=W, height=H, command=disconnect).place(x=0, y=y6)
    flag()
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

