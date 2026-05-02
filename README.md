# Actividad-5.1-Control-de-posici-n-

El objetivo de esta actividad es lograr que un robot diferencial en simulación se desplace desde una posición inicial hasta unas coordenadas `hxd` y `hyd` utilizando un controlador de lazo cerrado de tipo proporcional con diferentes ganancias fijas para velocidad linear y angular. 

1. Se establecieron los parámetros iniciales de tiempo para la simulación que sería el tiempo inicial de 0, el tiempo de muestreo `ts` de 0.1 y un tiempo final `tf` de 10. También se define su posición inicial x y y en `0,0` y su orientación que sería de `pi/2`. Después se definen las coordenadas objetivo `(hxd, hyd)`.
2. En cada iteración del bucle `for`, el sistema mide la diferencia entre la posición meta y la posición actual del robot. Esto genera un vector de error `he= [hxe(k);hye(k)];`. También, se calcula la magnitud del error mediante el teorema de pitagoras: `Error(k)= sqrt(hxe(k)^2 +hye(k)^2);`.
3. Se define una matriz Jacobiana la cual relaciona las velocidades del punto de control del robot con las velocidades de entrada. Esta matriz depende de la orientación actual del robot.

```
J=[cos(phi(k)) -sin(phi(k));... %Matriz de rotación en 2D
       sin(phi(k)) cos(phi(k))];
```
4. Se establece una matriz de ganancias la cual su diagonal contiene las ganancias para la velocidad linear en la posición `(0,0)` y la ganancia de la velocidad angular en la posición `(1,1)`. El control calcula las velocidades de referencia necesarias para minimizar el error utilizando la pseudoinversa de la matriz Jacobiana: `qpRef= pinv(J)*K*he;`. De este cálculo salen las velocidades linear y angular que el robot debe ejecutar en ese instante de tiempo. Al cambiar las ganancias de la matriz, se pudo observar que si es muy baja no alcanza el objetivo mientras que si es muy alta, provoca oscilaciones.
5. Las velocidades calculadas se aplican al robot. Mediante integración numérica con el método de Euler, se calculan la nueva orientación y la nueva posición en x y y para el siguiente instante de tiempo.

Para mejorar los resultados y no depender de sintonizar las ganancias por cada ejecución del programa dependiendo de que tan cerca o lejano está la siguiente coordenada, se planteó una estrategia adaptativa la cual modifica las ganancias dinámicamente según la distancia al objetivo. Para aquello, se adaptó el código con lo siguiente:

1. En lugar de una sola ganancia, se definieron tres parámetros antes de iniciar el ciclo de control:
   - `K_max` que es la ganancia máxima que tomará el sistema cuando el robot esté lejos del objetivo
   - `K_min` que es la ganancia mínima cuando el robot esté a punto de llegar
   - `alpha` que es el factor que determina qué tan rápido decae la ganancia de la máxima a la mínima
2. Dentro del bucle, después de calcular la magnitud del error de posición, se implementó una función exponencial que evalúa la ganancia en tiempo real: `k_adapt = K_min + (K_max - K_min) * (1 - exp(-alpha * Error(k)));`. Esto hace que cuando el error es grande, el término exponencial tiende a 0, haciendo que `k_adapt` se acerque a `k_max` y si es casi 0, se acerca más a `k_min`.
3. La matriz de ganancias de la estrategia anterior deja de ser constante y se reescribe en cada iteración utilizando el valor de `k_adapt`. 
4. El resto del proceso es igual, sin embargo, el comportamiento es distinto debido a que se puede observar como la aceleración inicial es rápida y una desaceleración suave cuando se acerca al objetivo, eliminando los movimientos bruscos y optimizando el tiempo de llegada. 
