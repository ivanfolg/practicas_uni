from controller import Robot
from controller import Camera
import time
import sys
import threading
import numpy as np

MAX_SPEED = 8
TIME_STEP = 24
COMPORTAMIENTO=0
W=0 #variables de la camara para que se calculen solo una vez y sea mas eficiente
H=0
w2=0
w3=0
w5=0
h2=0
h3=0
parar= 1 
lock = threading.Lock()
LEFT=0
RIGHT=0
MAX_INFRA=600
MIN_INFRA=160
VEL_LENTA=2
VEL_MEDIA=4
MIN_ULTRA=0.3
MAX_COMP=4

def init_devices(timeStep):
    robot = Robot()
    # Obtener dispositivos correspondientes a las ruedas.
    leftWheel = robot.getDevice('left wheel motor')
    rightWheel = robot.getDevice('right wheel motor')
    # Utilizamos velocidad, establecemos posición a infinito.
    leftWheel.setPosition(float('inf'))
    rightWheel.setPosition(float('inf'))
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)

    # Obtener y activar el dispositivo de la cámara    
    camera = robot.getDevice('camera')
    camera.enable(timeStep*10)

    # Activar otros sensores necesarios
    n_ultrasonic=5
    ultrasonic_sensors_names= [
    'left ultrasonic sensor', 'front left ultrasonic sensor', 'front ultrasonic sensor', 
    'front right ultrasonic sensor',
    'right ultrasonic sensor']
    ultrasonic_sensors=[]
    for i in range(0,n_ultrasonic):
        ultrasonic_sensors.append(robot.getDevice(ultrasonic_sensors_names[i]))
        ultrasonic_sensors[i].enable(timeStep)
    
    n_infrared=8
    infrared_sensors_names=[
    'rear left infrared sensor', 'left infrared sensor', 'front left infrared sensor', 'front infrared sensor',
    'front right infrared sensor', 'right infrared sensor', 'rear right infrared sensor', 'rear infrared sensor']
    infrared_sensors=[]
    for i in range(0,n_infrared):
        infrared_sensors.append(robot.getDevice(infrared_sensors_names[i]))
        infrared_sensors[i].enable(timeStep)
    
    # gyro=robot.getDevice('gyro')
    # gyro.enable(timeStep)
    global W, H, w, w2,w3,w5,h2,h3
    W = camera.getWidth()
    H = camera.getHeight()
    w = int(np.floor(W/5))
    h = int(np.floor(H/7))
    w2=w*2
    w3=w*3
    w5=w*5
    h2=h*3
    h3=h*4
    
    return robot, camera, leftWheel, rightWheel, infrared_sensors, ultrasonic_sensors#, gyro


def process_image():

    global W, H, w, w2, w3, w5, h2, h3, cameraData
    image = np.frombuffer(cameraData, np.uint8).reshape((H, W, 4))
    izq=0
    med=0
    der=0
    #color buscado perfecto 0,255,0
    MAXAZUL=40
    MINVERDE=170
    MAXROJO=50
    for x in range(h2,h3):  
        for y in range(0, W):
            if image[x,y,0]<MAXAZUL and image[x,y,1]>MINVERDE and image[x,y,2]<MAXROJO:
                if y<w2:
                    izq+=1
                elif w2<y and y<w3:
                    med+=1
                elif w3<y and y<w5:
                    der+=1
    media=int(np.mean(image[:,240:250,1]))
    return [izq, med, der, media]


def behaviour01(comp): #buscar pared andando en linea recta
    #ultrasonico cuanto mas lejos valores mas altos entre 0 e 2
    #infrarojo cuanto mas lejos valores mas bajos (max 25cm)
    #cuando los ultrasonicos dejan de medir y los infra empiezan a ver la pared
    global COMPORTAMIENTO, parar
    global RLInf, LInf,FLInf,FInf, FRInf, RInf,LUlt,FLUlt,FUlt,RUlt
    global MIN_INFRA, MAX_INFRA, MIN_ULTRA, VEL_MEDIA
    while parar == 1:
        # COMPORTAMIENTO<=comp and
        if FInf < MIN_INFRA and FLInf < MIN_INFRA and FRInf < MIN_INFRA and LUlt < MIN_ULTRA and RLInf < MIN_INFRA:
            x=(FUlt/1.2)
            change(comp, comp, (MAX_SPEED-1)*x,(MAX_SPEED-1)*x)
    
def behaviour02(comp):#seguir pared dejando esta a la izquierda
    global COMPORTAMIENTO, parar
    global RLInf, LInf,FLInf,FInf, FRInf, RInf,LUlt,FLUlt,FUlt,RUlt
    global MIN_INFRA, MIN_ULTRA, VEL_MEDIA, VEL_LENTA
    MARGEN_ERROR_INFRA=0.02
    porc_seg=0.25
    ultraSonicoMedio=0.45
    while parar == 1:  
        if FLInf > (MIN_INFRA-MIN_INFRA*porc_seg) and FLInf < MAX_INFRA and LInf > MIN_INFRA and LInf < MAX_INFRA and RLInf < MAX_INFRA and RLInf > (MIN_INFRA-MIN_INFRA*porc_seg):
            if ((FLInf - FLInf*MARGEN_ERROR_INFRA) <= RLInf) and ((FLInf + FLInf*MARGEN_ERROR_INFRA) >= RLInf):
                change(comp, comp, VEL_MEDIA,VEL_MEDIA)
            elif FLInf < RLInf:
                change(comp, comp, VEL_MEDIA-1.5,VEL_MEDIA)
            elif RLInf < FLInf:
                change(comp, comp, VEL_MEDIA,VEL_MEDIA-1.5)
        elif LInf > MIN_INFRA and LInf < MAX_INFRA and FLInf < MIN_INFRA and RLInf > (MIN_INFRA-MIN_INFRA*porc_seg):
            change(comp, comp, VEL_MEDIA-1.3, VEL_MEDIA+1.35)
        elif LUlt > ultraSonicoMedio and LInf < MIN_INFRA and FLInf < MIN_INFRA:
            change(comp, comp, VEL_MEDIA-3, VEL_MEDIA+1)
        else:
            change(0, comp, -100, -100)
def behaviour03(comp):
    global w2, w3, w5, h2, h3, COMPORTAMIENTO, parar, MAX_COMP
    global RLInf, LInf,FLInf, FInf, FRInf, RInf,LUlt,FLUlt,FUlt,RUlt
    global VEL_MEDIA, MIN_INFRA, MAX_INFRA
    MAX_LATERAL=(h3-h2)*(w5-w3)
    MAX_MEDIO=(h3-h2)*(w3-w2)
    media1=-1
    media=-1
    it=0
    while parar == 1:  
        media2=media1
        media1=media
        izq, med, der, media=process_image()
        if it==2:
            it=0
            if media2 == media and (FInf < MIN_INFRA or FInf > MAX_INFRA):
                change(comp, comp, 1.5, -1)
                change(0, comp, -100, -100)
        else:
            it+=1
        if izq > MAX_LATERAL*0.05 or der > MAX_LATERAL*0.05 or med > MAX_MEDIO*0.05:
            if izq> med and izq>der:
                change(comp, comp, VEL_MEDIA-1.5, VEL_MEDIA)
            elif der> med and der>izq:
                change(comp, comp, VEL_MEDIA, VEL_MEDIA-1.5)
            else:
                change(comp, comp, VEL_MEDIA, VEL_MEDIA)
        if med > MAX_MEDIO*0.4:
            change(MAX_COMP, comp, 0, 0)
            parar = 0      

def behaviour04(comp):#posicionarse para seguir pared dejando esta a la izquierda
    #girar 90 grados para posicionarse usando en este caso sensores infrarojos
    #de los tres sensores infrarojos de la izq
    global COMPORTAMIENTO, parar, MAX_COMP
    global RLInf, LInf, FLInf, FInf, FRInf, RInf,LUlt,FLUlt,FUlt,RUlt
    global MIN_INFRA, MAX_INFRA, VEL_LENTA
    #tomar control en cualquiera de las condiciones del if
    uno=1
    while parar == 1:    
        if FInf > MIN_INFRA and FLInf > (MIN_INFRA-MIN_INFRA*0.25) and FRInf> (MIN_INFRA-MIN_INFRA*0.25) and RInf < MIN_INFRA: 
            change(comp, comp, uno, -uno)
        elif FInf > MIN_INFRA and FLInf > MIN_INFRA and FRInf> MIN_INFRA and LInf < MIN_INFRA:
            change(comp, comp, -uno, uno)
        elif FLInf>MAX_INFRA:
            #muy pegado o a punto de chocar girar der
            change(comp, comp, uno, -uno)
        elif FRInf>MAX_INFRA:
            #muy pegado o a punto de chocar girar izq
            change(comp, comp, -uno, uno)
        elif FInf > MAX_INFRA and FLInf > FRInf and FLInf > MIN_INFRA: 
            change(comp, comp, uno, -uno)
        elif FInf > MAX_INFRA and FLInf < FRInf and FRInf > MIN_INFRA:
            change(comp, comp, -uno, uno)
        elif FInf < MIN_INFRA and FUlt < MIN_ULTRA and FLInf > MIN_INFRA and FRInf > MIN_INFRA:
            change(comp, comp, -uno, -uno)
        elif LInf > MAX_INFRA and RLInf>MIN_INFRA:
            change(comp, comp, uno, -uno)
        if COMPORTAMIENTO==MAX_COMP:
            change(0, comp, -100, -100)

def change(nuevo_comp, solicitante, left, right):
    global lock, COMPORTAMIENTO, LEFT, RIGHT
    lock.acquire()
    try:
        if COMPORTAMIENTO <= nuevo_comp:
            COMPORTAMIENTO = nuevo_comp
            if left != -100:
                LEFT=left
                RIGHT=right
        elif COMPORTAMIENTO==solicitante and nuevo_comp==0:
            COMPORTAMIENTO = nuevo_comp
        # print(COMPORTAMIENTO)
    finally:
        lock.release()

robot, camera, leftWheel, rightWheel, infrared_sensors, ultrasonic_sensors =init_devices(TIME_STEP)
robot.step(TIME_STEP)
cameraData=camera.getImage()
RLInf=infrared_sensors[0].getValue()
LInf=infrared_sensors[1].getValue()
FLInf=infrared_sensors[2].getValue()
FInf=infrared_sensors[3].getValue()
FRInf=infrared_sensors[4].getValue()
RInf=infrared_sensors[5].getValue()

LUlt=ultrasonic_sensors[0].getValue()
FLUlt=ultrasonic_sensors[1].getValue()
FUlt=ultrasonic_sensors[2].getValue() 
FRUlt=ultrasonic_sensors[3].getValue() 
RUlt=ultrasonic_sensors[4].getValue() 
last_display_second=0
x1 = threading.Thread(target=behaviour01, args=(1,))
x2 = threading.Thread(target=behaviour02, args=(2,))
x3 = threading.Thread(target=behaviour03, args=(3,))
x4 = threading.Thread(target=behaviour04, args=(4,))

x1.start()
x2.start()
x3.start()
x4.start()
while robot.step(TIME_STEP) != -1 or parar==0:
    display_second=robot.getTime()
    if display_second != last_display_second:
        last_display_second = display_second 
    cameraData=camera.getImage()
    RLInf=infrared_sensors[0].getValue()
    LInf=infrared_sensors[1].getValue()
    FLInf=infrared_sensors[2].getValue()
    FInf=infrared_sensors[3].getValue()
    FRInf=infrared_sensors[4].getValue()
    RInf=infrared_sensors[5].getValue()
    
    LUlt=ultrasonic_sensors[0].getValue()
    FLUlt=ultrasonic_sensors[1].getValue()
    FUlt=ultrasonic_sensors[2].getValue() 
    FRUlt=ultrasonic_sensors[3].getValue() 
    RUlt=ultrasonic_sensors[4].getValue() 
    rightWheel.setVelocity(RIGHT)
    leftWheel.setVelocity(LEFT)


parar=0
rightWheel.setVelocity(0)
leftWheel.setVelocity(0)
x1.join()
x2.join()
x3.join()
x4.join()