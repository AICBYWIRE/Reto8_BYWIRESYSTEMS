# Reto8_BYWIRESYSTEMS
06/03/25 el programa consigue leer y recibir datos de diferentes partes, además se tiene una GUI.

v0.00 (06/03/2025) Esta versión recibe los datos de 0x290 y 0x292, y los muestra por pantalla de forma que sean entendibles. Además, se envían mensajes CAN a la ECU vía 0x298, en el modo de steer angle, es decir, se le da un ángulo y una ganancia y la cremallera se va hasta ese ángulo. Tiene un menú para interactuar. 

v0.1. En esta versión se reciben datos por pantalla y no se vuelven a enviar hasta que no hay un 5% de diferencia con el mensaje anterior para no saturar la consola. 

v1 (27/03/2025). En esta versión, se cambia la base y ahora se leen analógicamente mensajes de Voltaje de un sensor tipo hall en el IO35, y el voltaje de la fuente o batería en el IO36. Por otro lado, se leen los mensajes 290h y 292h del SBW y se representan por pantalla para poder hacer un debug con "SERIAL STUDIO". En este punto del código somos capaces de recibir vía CAN y vía Entradas analógicas para hacer un estudio profundo. 

v1.1  Añadido un encoder de posición que lee posición y con un pulsador, ganancia del volante. Simula el volante mientras llega el servomotor. 

v1.1.1 (31/03/2025) Añadido clock con millis para que el envio y recepción sean cada x milisegundos. 

v1.1.2  Eliminada cualquier referencia al CANBUS

v1.1.3 Recodeado todo lo referente a CANBUS en una única función

v1.1.4 Ahora lee y envía CANBUS pero no llega al bucle de envío de datos analógicos. 

v1.2 Lectura, recepción y envío de can y analógico, correcto. 

v1.2.1 Reorganización menor de código. Funciona estable. 

v1.2.2 Reestructuración código(AI) // SERIAL STUDIO: Plottin.json

v1.3 (02/04/2025) Reestructuración de las variables de entrada y salida. Ahora todos los calculos son sobre el voltaje, la corriente, torque_salida y torque_volante son solo escalados entre V,I,T. 

v1.3.1 Reordenación de datos. Inclusión del voltaje_volante como una escala del torque_salida.

v1.3.1.1 debug

v1.3.2 (08/04/2025) Añadido una medida "suave" de voltaje, intensidad, torque. Multimetro-like. 

v1.3.2.1 Suavizado aún más la salida de "multimetro"

v1.4 Primera iteración metiendo la lógica de cuando el volante gira a derechas o a izquierdas, ya que el torque tiene que ser positivo o negativo dependiendo del momento. 

v1.4.1 (10/04/2025) Pequeños cambios. Última versión antes de integrar el servomotor del volante. 

v2.0 (06/05/2025) Añadido el control del DYNAMIXEL. Servomotor para el FFB. Modo control de corriente. 

v2.1 (14/05/2025) Cambiado el control del encoder, ahora la posición ya se lee desde el el propio DYNAMIXEL. 

v2.1.1 (15/05/2025) Añadido el límite virtual a ambos lados del fin de carrera mecánico. 

v2.2.2 Reestructuración del código en bloques. Aligeramiento del main()

v2.2.3 (16/05/2025) Añadido el autoretorno como k*w. Fallo en la zona cercana a 0º. 

v 2.2.4 (20/05/2025) Cambiada la lógica de giro del enconder al propio DYNAMIXEL. (Retraso elevado, revisar filtros)

v 2.2.5 (21/05/2025) Activada lógica de torque y recentrado pudiendose activar y desactivar 

v2.2.6 (02/06/2025) Lógica de recentrado cuadrática

v2.2.6.1 Reescalado de la lógica de recentrado.

v2.2.7 Zona muerta de recentrado. Logica lineal. 

v2.3 (04/06/2025) PRUEBA DE CODIGO CON CLAUDEAI. REESTRUCTURACIÓN Y MEJORA. :)

v2.3.1 PRUEBA DE CODIGO CON DEEPSEEK :)

v2.3.2 (05/06/2025) Código de CLAUDEAI con modificaciones en la zona de error de sensores.







© 2025 Marco Molinari Pérez | Automotive Intelligence Center (AIC)