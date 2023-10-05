import cv2
import numpy as np

img=cv2.imread('materialFutbol/2.jpg', cv2.IMREAD_UNCHANGED)
img=np.array(img)
b,g,r=cv2.split(img)
f, c, d=img.shape
cv2.imshow("proceso", img)
cv2.waitKey()
################################################################################################################################################
################################################################################################aplicado de una mascara para el verde en base a la mediana de color de la imagen
################################################################################################paso de un filtro para eliminar los elementos pequeños
################################################################################################paso de un filtro de medias para suavizar los bordes
medianaR=np.median(r)
medianaV=np.median(g)
medianaA=np.median(b)
porc=0.4
rojo_bajo=np.full((f,c), medianaR-medianaR*porc, dtype=np.uint8)
rojo_alto=np.full((f,c), medianaR+medianaR*porc, dtype=np.uint8)
verde_bajo=np.full((f,c), medianaV-medianaV*porc, dtype=np.uint8)
verde_alto=np.full((f,c), medianaV+medianaV*porc, dtype=np.uint8)
azul_bajo=np.full((f,c), medianaA-medianaA*porc, dtype=np.uint8)
azul_alto=np.full((f,c), medianaA+medianaA*porc, dtype=np.uint8)
bajo=np.zeros((f, c, d), dtype=np.uint8)
alto=np.zeros((f, c, d), dtype=np.uint8)
bajo[:,:,0]+=azul_bajo
bajo[:,:,1]+=verde_bajo
bajo[:,:,2]+=rojo_bajo
alto[:,:,0]+=azul_alto
alto[:,:,1]+=verde_alto
alto[:,:,2]+=rojo_alto

campo=cv2.inRange(img, bajo, alto)
cv2.imshow("proceso", campo)
cv2.waitKey()

campo2=cv2.morphologyEx(campo, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_CROSS, (7,7)))/255
cv2.imshow("proceso", campo2)
cv2.waitKey()

salida1=np.zeros((f, c, d), dtype=np.uint8)
salida1[:,:,0]=img[:,:,0]*campo2
salida1[:,:,1]=img[:,:,1]*campo2
salida1[:,:,2]=img[:,:,2]*campo2
cv2.imshow("proceso", salida1)
cv2.waitKey()

outMedian=cv2.medianBlur(salida1, 9)
cv2.imshow("proceso", outMedian)
cv2.waitKey()
################################################################################################################################################################################################
################################################################################################campo personas con mask-suavizado-contornos-filtrado de contornos

#tras filtrar la mascara nos quedamos el trozo mayor que corresponde con el campo, y procesaremos los pequeños para detectar huecos en este que seran los jugadores

##################################################################################################################################################################################################
#buscamos los contornos de la imagen recortada

gray = cv2.cvtColor(outMedian, cv2.COLOR_BGR2GRAY)
contornos, propiedades = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
maxArea=0

#nos quedamos el trozo mayor que representa el campo y hacemos una mascara, esto para tener un recorte sin huecos donde estarian los juegadores y que sera la
#que se muestre mas tarde al terminar
for i in range(len(contornos)):
    tmp = cv2.contourArea(contornos[i])
    if tmp > maxArea:
        maxArea = tmp
        masGrande = i

mascaraCampo=np.zeros((f, c, d), dtype=np.uint8) #campo
cv2.drawContours(mascaraCampo, [contornos[masGrande]], -1,(255, 255, 255), thickness=cv2.FILLED)
img=cv2.bitwise_and(img, mascaraCampo)
cv2.imshow("proceso", img)
cv2.waitKey()
##################################################################################################################################################################################################

gray = cv2.cvtColor(outMedian, cv2.COLOR_BGR2GRAY)
cv2.imshow("proceso", gray)
cv2.waitKey()

resaltada=cv2.addWeighted(gray, 1.1, cv2.medianBlur(gray, 9), 0.9, 0)
cv2.imshow("proceso", resaltada)
cv2.waitKey()
v = np.median(resaltada)
canny=cv2.Canny(resaltada, 0, v*0.12)
cv2.imshow("proceso", canny)
cv2.waitKey()

lines=cv2.HoughLinesP(canny, 1, np.pi/180, 150, minLineLength=f*0.25, maxLineGap=f*0.05)
if lines is not None: #si se detectan lineas filtramos las horizontales y semi-horizontales eliminandolas
    conservar=[]
    ascendentes=0
    for line in lines:
        x1, y1, x2, y2 = line[0]
        anguloLinea = np.rad2deg(np.arctan2(y2-y1, x2-x1)) #los angulos salen desde -0 a 0 pasando por -90 que es la vertical, los ascendentes son 0 a 90 los descendentes -0 a -90
        if anguloLinea < -12 or anguloLinea > 12: #descartamos los horizontales y nos quedamos el resto en "conserver"
            conservar.append([x1, y1, x2, y2, anguloLinea])
            if anguloLinea > 0:
                ascendentes+=1 #cuento los de un sentido para descartar luego los del sentido minoritario
    if ascendentes/len(conservar) > 0.5: #mas ascendentes que descendentes me quedo con la mayoria
        for line in conservar:
            if line[4] > 0:
                cv2.line(img, (line[0], line[1]), (line[2], line[3]), (35, 220, 255), 3)
    else:
        for line in conservar:
            if line[4] < 0:
                cv2.line(img, (line[0], line[1]), (line[2], line[3]), (35, 220, 255), 3)
cv2.imshow("proceso", img)
cv2.waitKey()
##################################################################################################################################################################################################
#buscamos y marcamos todos los jugadores
for i in range(len(contornos)):
    if propiedades[0][i][3] >= 0: #solo los que son contornos dentro de otros contornos es decir jugadores dentro del campo
        x, y, w, h = cv2.boundingRect(contornos[i])
        if (h / w) > 1.2 and (h / w) < 3.6:
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
##################################################################################################################################################################################################
#se muestra el resultado final
cv2.imshow("proceso", img)
cv2.waitKey()
