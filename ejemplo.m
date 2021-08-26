%ejemplo para implementar metodos forward kinematics e inverse kinematics
a=Bycicle(2,3);%inicialización de la clase con los parámetros wheel base y wheel radius
b=a.calcForwardKinematics(4,pi/4);%calculo de forward kinematics con velocidad angular 4 y angulo delta pi cuartos
[c,d]=a.calcInverseKinematics(b);%cálculo de inverse kinematics
