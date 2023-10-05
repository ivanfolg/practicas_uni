#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/camera.h>
#include <stdio.h>
#include <stdlib.h>
//velocidad de movimiento 20
#define MAX_SPEED 20
//valores de aprendizaje y sensores
#define ALPHA 0.5
#define GAMMA 0.5
#define MAX_ITER 500
#define TIME_STEP 24
#define MIN_INFRA 500
#define MAX_INFRA 750
#define MIN_INFRA_CHOCAR 450


//DATOS EXTRAIDOS DE LA PRACTICA ANTERIOR
const float andar_X=20;//distancia, en mm, que recorre el robot en linea recta de cada pasada
const float angulo_de_giro=0.3490659;//angulo que girara el robot, en radianes, 20 en deg

const int radio_rueda=21; // en mm
const float incr=andar_X/radio_rueda;//incremento que se suma al valor medido por los encoders antes de moverse

const float radio_entre_ruedas=108.24586;
//incremento que se suma al valor medido por los encoders antes de girar
const float incr_giro=(angulo_de_giro*radio_entre_ruedas)/radio_rueda;
const float incr_esquivar=(0.738999999*radio_entre_ruedas)/radio_rueda; //aprox angulo de 40 grados en rad
// que gira solo para no chocarse

float gli, gri, gfli, gfri; //sensores inferiores
float gli_prev, gri_prev, gfli_prev, gfri_prev; //sensores inferiores valores medidos antes de la accion
int moviendose = 0; //0= parado con lo que calcula el siguiente movimiento y entrena
float Q[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
int iterCount = 0;//cuenta iteraciones para saber si esta entrenado
int estado=-1;
int accion=0;

#define NUMBER_OF_ULTRASONIC_SENSORS 5
static const char *ultrasonic_sensors_names[NUMBER_OF_ULTRASONIC_SENSORS] = {
  "left ultrasonic sensor", "front left ultrasonic sensor", "front ultrasonic sensor", "front right ultrasonic sensor",
  "right ultrasonic sensor"};
WbDeviceTag ultrasonic_sensors[5];

#define NUMBER_OF_INFRARED_SENSORS 12
static const char *infrared_sensors_names[NUMBER_OF_INFRARED_SENSORS] = {
  // turret sensors
  "rear left infrared sensor", "left infrared sensor", "front left infrared sensor", "front infrared sensor",
  "front right infrared sensor", "right infrared sensor", "rear right infrared sensor", "rear infrared sensor",
  // ground sensors
  "ground left infrared sensor", "ground front left infrared sensor", "ground front right infrared sensor",
  "ground right infrared sensor"};
WbDeviceTag infrared_sensors[12]; 
WbDeviceTag left_motor, right_motor, encoderL, encoderR;

void inicializar_robot(){
	wb_robot_init();

	int i;
		
	for (i = 0; i < 5; ++i) {
	ultrasonic_sensors[i] = wb_robot_get_device(ultrasonic_sensors_names[i]);
	wb_distance_sensor_enable(ultrasonic_sensors[i], TIME_STEP);
	}
	
	for (i = 0; i < 12; ++i) {
	infrared_sensors[i] = wb_robot_get_device(infrared_sensors_names[i]);
	wb_distance_sensor_enable(infrared_sensors[i], TIME_STEP);
	}
	
	left_motor = wb_robot_get_device("left wheel motor");
	right_motor = wb_robot_get_device("right wheel motor");
	encoderL = wb_robot_get_device("left wheel sensor");
	encoderR = wb_robot_get_device("right wheel sensor");
	wb_position_sensor_enable(encoderL, TIME_STEP);
	wb_position_sensor_enable(encoderR, TIME_STEP);
	wb_motor_set_position(left_motor, 0);
	wb_motor_set_position(right_motor, 0);
	wb_motor_set_velocity(left_motor, 0.0);
	wb_motor_set_velocity(right_motor, 0.0);
	
}  

int mirarSensores(){
	gli_prev=gli;
	gri_prev=gri;
	gfli_prev=gfli;
	gfri_prev=gfri;
	gli=wb_distance_sensor_get_value(infrared_sensors[8]);
	gfli=wb_distance_sensor_get_value(infrared_sensors[9]);
	gfri=wb_distance_sensor_get_value(infrared_sensors[10]);
	gri=wb_distance_sensor_get_value(infrared_sensors[11]);
	
	if(gfli > MAX_INFRA && gri < MIN_INFRA)//abandona linea por izq
		return 1;
	else if(gfri > MAX_INFRA && gli < MIN_INFRA)//abandona por la der
		return 2;
	else return 0; //recto
}

int maximizar(int est){
	if ((Q[est][0] >= Q[est][1] ) && (Q[est][0] >= Q[est][2] )){
		return 0;//accion numero 0 es la mejor
	}else if (Q[est][1] >= Q[est][2]){
		return 1; //accion numero 1 es la mejor
	}else{
		return 2; //accion numero 2 es la mejor
	}
}

void siguienteAccion(){
	int r=rand() % MAX_ITER; //num random entre 0 y el total de iter de entrenamiento
	if (iterCount < r){//accion aleatorizada 
		accion = rand() % 3;//genera un numero entre 0 y 2 ambos incluidos 
	}else{//accion optima
		accion = maximizar(estado);
	}
}

void aprender(){
	float ref = 0; //refuerzo
	int nuevoEstado=mirarSensores();
	
	if (gli_prev > MAX_INFRA && gli < MIN_INFRA) ref+=1; //si antes no y ahora si ref positivo
	else if (gli_prev > MAX_INFRA && gli > MAX_INFRA) ref-=0.25; //si antes no y ahora tampoco negativo
	else if (gli_prev < MIN_INFRA && gli > MAX_INFRA) ref-=0.25; //si antes si y ahora no negativo
	else if (gli_prev < MIN_INFRA && gli < MIN_INFRA) ref+=1; //si antes si y ahora tambien positivo
	
	if (gri_prev > MAX_INFRA && gri < MIN_INFRA) ref+=1; //si antes no y ahora si ref positivo
	else if (gri_prev > MAX_INFRA && gri > MAX_INFRA) ref-=0.25; //si antes no y ahora tampoco negativo
	else if (gri_prev < MIN_INFRA && gri > MAX_INFRA) ref-=0.25; //si antes si y ahora no negativo
	else if (gri_prev < MIN_INFRA && gri < MIN_INFRA) ref+=1; //si antes si y ahora tambien positivo
	
	if (gfli_prev > MAX_INFRA && gfli < MIN_INFRA) ref+=0.25; //si antes no y ahora si ref positivo
	else if (gfli_prev > MAX_INFRA && gfli > MAX_INFRA) ref-=0.25; //si antes no y ahora tampoco negativo
	else if (gfli_prev < MIN_INFRA && gfli > MAX_INFRA) ref-=0.25; //si antes si y ahora no negativo
	else if (gfli_prev < MIN_INFRA && gfli < MIN_INFRA) ref+=0.25; //si antes si y ahora tambien positivo
	
	if (gfri_prev > MAX_INFRA && gfri < MIN_INFRA) ref+=0.25; //si antes no y ahora si ref positivo
	else if (gfri_prev > MAX_INFRA && gfri > MAX_INFRA) ref-=0.25; //si antes no y ahora tampoco negativo
	else if (gfri_prev < MIN_INFRA && gfri > MAX_INFRA) ref-=0.25; //si antes si y ahora no negativo
	else if (gfri_prev < MIN_INFRA && gfri < MIN_INFRA) ref+=0.25; //si antes si y ahora tambien positivo
	

	Q[estado][accion] = (1-ALPHA)*Q[estado][accion] + ALPHA*(ref + GAMMA*Q[nuevoEstado][maximizar(nuevoEstado)] - Q[estado][accion]);
	iterCount++;
	
	/*para ver los datos de la tabla segun se va creando*/
	printf("iter %d \n", iterCount);
	for(int i=0;i<=2;i++){
		for(int j=0;j<=2;j++){
			printf(" %f ", Q[i][j]);
		}
		printf("\n");
	}
	printf("\n\n");
	
	estado = nuevoEstado; 
}

int main(int argc, char **argv) {
	inicializar_robot();
	//mirar situacion actual de sensores en el momento inicial
	estado=mirarSensores();
	float posL_antes=0;
	float posR_antes=0;
	float posL=-1;
	float posR=-1;
	float posL_desp=0;
	float posR_desp=0;
	// main loop
	while (wb_robot_step(TIME_STEP) != -1) {	
		//seleccionar accion de la tabla
		if(!moviendose){
			siguienteAccion();
			posL_antes=wb_position_sensor_get_value(encoderL);
			posR_antes=wb_position_sensor_get_value(encoderR);
			switch(accion){
				case 0:{
					 // /*recto*/
					posL_desp=posL_antes+incr;
					posR_desp=posR_antes+incr;
				}; break;
				case 1:{
					// /*gira der*/
					posL_desp=posL_antes+incr;
					posR_desp=posR_antes-incr;
				}; break;
				case 2:{
					// /*girar izq*/
					posL_desp=posL_antes-incr;
					posR_desp=posR_antes+incr;
				}; break;
			}
			wb_motor_set_position(left_motor, posL_desp);
			wb_motor_set_position(right_motor, posR_desp);
			moviendose=1;
		}
		//ejecutarla
		wb_motor_set_velocity(left_motor, MAX_SPEED);
		wb_motor_set_velocity(right_motor, MAX_SPEED);
		posR_antes=posR;
		posL_antes=posL;
		posL=wb_position_sensor_get_value(encoderL);
		posR=wb_position_sensor_get_value(encoderR);
		if (posL == posL_antes || posR == posR_antes) moviendose=0;
		
		if(!moviendose){
			// si estamos entrenando
			if (iterCount < MAX_ITER){
				/*aprender*/
				aprender();
			}
			iterCount++;
			//mirar situacion actual de sensores
			estado = mirarSensores();
			// mirar si chocamos frontalmente y esquivar de ser el caso
			if(wb_distance_sensor_get_value(infrared_sensors[2]) > MIN_INFRA_CHOCAR || 
				wb_distance_sensor_get_value(infrared_sensors[3]) > MIN_INFRA_CHOCAR ||
				wb_distance_sensor_get_value(infrared_sensors[4]) > MIN_INFRA_CHOCAR){
				posL_antes=wb_position_sensor_get_value(encoderL);
				posR_antes=wb_position_sensor_get_value(encoderR);
				posL_desp=posL_antes-incr_esquivar;
				posR_desp=posR_antes+incr_esquivar;
				wb_motor_set_position(left_motor, posL_desp);
				wb_motor_set_position(right_motor, posR_desp);
				moviendose=1;
			}
		}
	}

	wb_robot_cleanup();

	return EXIT_SUCCESS;
}
