# Reto8_BYWIRESYSTEMS
06/03/25 el proagrma consigue leer y recibir datos de diferentes partes, además se tiene una GUI.

v0.00 (06/03/2025) Esta versión recibe los datos de 0x290 y 0x292, y los muestra por pantalla de forma que sean entendibles. Además, se envían mensajes CAN a la ECU vía 0x298, en el modo de steer angle, es decir, se le da un ángulo y una ganancia y la cremallera se va hasta ese ángulo. Tiene un menú para interactuar. 

v0.1. (06/03/2025). En esta versión se reciben datos por pantalla y no se vuelven a enviar hasta que no hay un 5% de diferencia con el mensaje anterior para no saturar la consola. 

v1 (27/03/2025). En esta versión, se cambia la base y ahora se leen analógicamente mensajes de Voltaje de un sensor tipo hall en el IO35, y el voltaje de la fuente o batería en el IO36. Por otro lado, se leen los mensajes 290h y 292h del SBW y se representan por pantalla para poder hacer un debug con "SERIAL STUDIO". En este punto del código somos capaces de recibir vía CAN y vía Entradas analógicas para hacer un estudio profundo. 

v1.1 (27/03/2025) Añadido un encoder de posición que lee posición y con un pulsador, ganancia del volante. Simula el volante mientras llega el servomotor. 







© 2025 Marco Molinari Pérez | Automotive Intelligence Center (AIC)