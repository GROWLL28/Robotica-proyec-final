#!/usr/bin/env python3
#se importan las librerías necesarias para el funcionamiento de este nodo. 
import rospy
import tty
import sys
import pytesseract
import cv2
import numpy as np
import time
import termios
import keyboard
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from PIL import Image

s='s'
w='w'
a='a'
d='d'
m='m'

R1=[s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,m]
R2=[w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,a,m]
R3=[d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,d,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,m]	
			
	
	
def robot_manipulador_planer(): #se define la función que publicará al tópico de la velocidad de turtle bot


	pub = rospy.Publisher('arm_color', String, queue_size=10) #declaramos la variable que publicará las velocidades
	pub2 = rospy.Publisher('robot_cmdVel', Twist, queue_size=10) #declaramos la variable que publicará las velocidades

	rospy.init_node('robot_manipulador_ping_pong', anonymous=True)  #inicializamos el nodo
	rate = rospy.Rate(10) #definimos la frecuencia de actualización

	filedescriptors = termios.tcgetattr(sys.stdin) 
	tty.setcbreak(sys.stdin)
	
	menssage="Esperando color"
	print(menssage)
	X=input()
	print(X)
	Estado=0
	Camino = []
	if Estado==0:
		for i in R1:  
			message = Twist()
			message.linear.x = 0
			message.linear.y = 0
			message.linear.z = 0
			message.angular.x = 0
			message.angular.y = 0
			message.angular.z = 0
			pub2.publish(message)

			
			if i == "s":			#se ralizan condiciones para evaluar si la letra oprimida es alguna de las de manejo
							#(w,a,s,d). En caso de ser así, se efectíua el cambio en el tópico a través del nodo.
				message.linear.x = -120
				message.linear.y = 0
				message.linear.z = 0		
				pub2.publish(message)

			elif i == "w":
				message.linear.x = 120
				message.linear.y = 0
				message.linear.z = 0
				pub2.publish(message)
				
			elif i == "a":
				message.angular.x = 0
				message.angular.y = 0
				message.angular.z = 120		
				pub2.publish(message)

			elif i == "d":
				message.angular.x = 0
				message.angular.y = 0
				message.angular.z = -120
				pub2.publish(message)	
			elif i == "m":
				break 
			Camino.append(i) #se anexa cada tecla oprimida a la lista
			np.savetxt('lista.txt',Camino, fmt="%s") #se guarda la lista en un archivo.txt llamado "lista"
	
			print(Camino)
			rospy.loginfo(message) #se envía la actualización de las compenentes de las velocidades al tópico
			rate.sleep()					
		mo= int(input("Siguiente")) 
		Estado=mo
	if Estado==1:
		cap = cv2.VideoCapture(0)
		#Azul
		azulBajo = np.array([100,100,20],np.uint8)
		azulAlto = np.array([125,255,255],np.uint8)
		#amarillo
		amarilloBajo = np.array([15,100,20],np.uint8)
		amarilloAlto = np.array([45,255,255],np.uint8)
		#rojo 
		redBajo1 = np.array([0,100,20],np.uint8)
		redAlto1 = np.array([5,255,255],np.uint8)
		redBajo2 = np.array([175,100,20],np.uint8)
		redAlto2 = np.array([179,255,255],np.uint8)
		
		if X=="azul":
			
			pub.publish(X)		
			while Estado==0:
			  ret,frame = cap.read()
			  if ret==True:
			    frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
			    mask = cv2.inRange(frameHSV,azulBajo,azulAlto)
			    contornos,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			    #cv2.drawContours(frame, contornos, -1, (255,0,0), 3)
			    for c in contornos:
			      area = cv2.contourArea(c)
			      if area > 3000:
			        M = cv2.moments(c)	
			        if (M["m00"]==0): M["m00"]=1
			        x = int(M["m10"]/M["m00"])
			        y = int(M['m01']/M['m00'])
			        if x<=200:
			        	message = Twist()
			        	message.linear.z = 1
			        	pub2.publish(message)
			        	mo= int(input("Siguiente")) 
			        	Estado=mo
			        	
			        	
			        if x>200 and x<=400:
			        	message = Twist()
			        	message.linear.z = 2
			        	pub2.publish(message)
			        	mo= int(input("Siguiente")) 
			        	Estado=mo
			        	
			        if x>400:
			        	message = Twist()
			        	message.linear.z = 3
			        	pub2.publish(message)
			        	mo= int(input("Siguiente")) 
			        	Estado=mo
			        		        
			        cv2.circle(frame, (x,y), 7, (0,255,0), -1)
			        font = cv2.FONT_HERSHEY_SIMPLEX
			        cv2.putText(frame, '{},{}'.format(x,y),(x+10,y), font, 0.75,(0,255,0),1,cv2.LINE_AA)
			        nuevoContorno = cv2.convexHull(c)
			        cv2.drawContours(frame, [nuevoContorno], 0, (255,0,0), 3)
			    #cv2.imshow('maskAzul',mask)
			    cv2.imshow('frame',frame)
			    if cv2.waitKey(1) & 0xFF == ord('s'):
			      break
			cap.release()
			cv2.destroyAllWindows()
		if X=="amarillo":
			pub.publish(X)	
			while Estado==0:
			  ret,frame = cap.read()
			  if ret==True:
			    frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
			    mask = cv2.inRange(frameHSV,amarilloBajo,amarilloAlto)
			    contornos,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			    #cv2.drawContours(frame, contornos, -1, (255,0,0), 3)
			    for c in contornos:
			      area = cv2.contourArea(c)
			      if area > 3000:
			        M = cv2.moments(c)
			        if (M["m00"]==0): M["m00"]=1
			        x = int(M["m10"]/M["m00"])
			        y = int(M['m01']/M['m00'])
			        if x<=200:
			        	message = Twist()
			        	message.linear.z = 1
			        	mo= int(input("Siguiente")) 
			        	Estado=mo
			        	
			        if x>200 and x<=400:
			        	message = Twist()
			        	message.linear.z = 2
			        	mo= int(input("Siguiente")) 
			        	Estado=mo
			        	
			        if x>400:
			        	message = Twist()
			        	message.linear.z = 3
			        	mo= int(input("Siguiente")) 
			        	Estado=mo
			        		
			        cv2.circle(frame, (x,y), 7, (0,0,255), -1)
			        font = cv2.FONT_HERSHEY_SIMPLEX
			        cv2.putText(frame, '{},{}'.format(x,y),(x+10,y), font, 0.75,(0,255,0),1,cv2.LINE_AA)
			        nuevoContorno = cv2.convexHull(c)
			        cv2.drawContours(frame, [nuevoContorno], 0, (0,255,255), 3)
			    #cv2.imshow('maskAzul',mask)
			    cv2.imshow('frame',frame)
			    if cv2.waitKey(1) & 0xFF == ord('s'):
			      break
			cap.release()
			cv2.destroyAllWindows()
			
		if X=="rojo":
			pub.publish(X)		
			while Estado==0:
			  ret,frame = cap.read()
			  if ret==True:
			    frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
			    maskRed1 = cv2.inRange(frameHSV, redBajo1, redAlto1)
			    maskRed2 = cv2.inRange(frameHSV, redBajo2, redAlto2)
			    maskRed = cv2.add(maskRed1, maskRed2)
			    contornos,_ = cv2.findContours(maskRed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			    #cv2.drawContours(frame, contornos, -1, (255,0,0), 3)
			    for c in contornos:
			      area = cv2.contourArea(c)
			      if area > 3000:
			        M = cv2.moments(c)
			        if (M["m00"]==0): M["m00"]=1
			        x = int(M["m10"]/M["m00"])
			        y = int(M['m01']/M['m00'])
			        if x<=200:
			        	message = Twist()
			        	message.linear.z = 1
			        	pub2.publish(message)
			        	Estado=2
			        	
			        if x>200 and x<=400:
			        	message = Twist()
			        	message.linear.z = 2
			        	mo= int(input("Siguiente")) 
			        	Estado=mo
			        	
			        if x>400:
			        	message = Twist()
			        	message.linear.z = 3
			        	mo= int(input("Siguiente")) 
			        	Estado=mo
			        	
			        cv2.circle(frame, (x,y), 7, (0,255,0), -1)
			        font = cv2.FONT_HERSHEY_SIMPLEX
			        cv2.putText(frame, '{},{}'.format(x,y),(x+10,y), font, 0.75,(0,255,0),1,cv2.LINE_AA)
			        nuevoContorno = cv2.convexHull(c)
			        cv2.drawContours(frame, [nuevoContorno], 0, (0,0,255), 3)
			    #cv2.imshow('maskAzul',mask)
			    cv2.imshow('frame',frame)
			    if cv2.waitKey(1) & 0xFF == ord('s'):
			      break
			cap.release()
			cv2.destroyAllWindows()	
					
	if Estado==2:
		for i in R2:  
			message = Twist()
			message.linear.x = 0
			message.linear.y = 0
			message.linear.z = 0
			message.angular.x = 0
			message.angular.y = 0
			message.angular.z = 0
			pub2.publish(message)

			
			if i == "s":			#se ralizan condiciones para evaluar si la letra oprimida es alguna de las de manejo
							#(w,a,s,d). En caso de ser así, se efectíua el cambio en el tópico a través del nodo.
				message.linear.x = -120
				message.linear.y = 0
				message.linear.z = 0		
				pub2.publish(message)

			elif i == "w":
				message.linear.x = 120
				message.linear.y = 0
				message.linear.z = 0
				pub2.publish(message)
				
			elif i == "a":
				message.angular.x = 0
				message.angular.y = 0
				message.angular.z = 120		
				pub2.publish(message)

			elif i == "d":
				message.angular.x = 0
				message.angular.y = 0
				message.angular.z = -120
				pub2.publish(message)	
			elif i == "m":
				break 
			Camino.append(i) #se anexa cada tecla oprimida a la lista
			np.savetxt('lista.txt',Camino, fmt="%s") #se guarda la lista en un archivo.txt llamado "lista"
	
			print(Camino)
			rospy.loginfo(message) #se envía la actualización de las compenentes de las velocidades al tópico
			rate.sleep()					
		mo= int(input("Siguiente")) 
		Estado=mo
	if Estado==3:
		
		cap = cv2.VideoCapture(0)

		leido, frame = cap.read()

		if leido == True:
			cv2.imwrite("foto.png", frame)
			print("Foto tomada correctamente")
		else:
			print("Error al acceder a la cámara")

		"""
			Finalmente liberamos o soltamos la cámara
		"""
		cap.release()
		
		im = Image.open("/home/robotica/catkin_ws1/foto.png")

		# Utilizamos el método "image_to_string"
		# Le pasamos como argumento la imagen abierta con Pillow
		texto = pytesseract.image_to_string(im)

		# Mostramos el resultado
		print(texto)
		mo= int(input("Siguiente")) 
		Estado=mo
	if Estado==4:
		for i in R3:  
			message = Twist()
			message.linear.x = 0
			message.linear.y = 0
			message.linear.z = 0
			message.angular.x = 0
			message.angular.y = 0
			message.angular.z = 0
			pub2.publish(message)

			
			if i == "s":			#se ralizan condiciones para evaluar si la letra oprimida es alguna de las de manejo
							#(w,a,s,d). En caso de ser así, se efectíua el cambio en el tópico a través del nodo.
				message.linear.x = -120
				message.linear.y = 0
				message.linear.z = 0		
				pub2.publish(message)

			elif i == "w":
				message.linear.x = 120
				message.linear.y = 0
				message.linear.z = 0
				pub2.publish(message)
				
			elif i == "a":
				message.angular.x = 0
				message.angular.y = 0
				message.angular.z = 120		
				pub2.publish(message)

			elif i == "d":
				message.angular.x = 0
				message.angular.y = 0
				message.angular.z = -120
				pub2.publish(message)	
			elif i == "m":
				break 
			Camino.append(i) #se anexa cada tecla oprimida a la lista
			np.savetxt('lista.txt',Camino, fmt="%s") #se guarda la lista en un archivo.txt llamado "lista"
	
			print(Camino)
			rospy.loginfo(message) #se envía la actualización de las compenentes de las velocidades al tópico
			rate.sleep()					
		mo= int(input("Siguiente")) 
		Estado=mo
	if Estado==5:

		message.linear.z = 4
		pub2.publish(message)	
if __name__ == '__main__': #se compilan las funciones definidas en le main
	try:

		robot_manipulador_planer()
	except rospy.ROSInterruptException:
		pass
