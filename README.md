# STM32ADC_PlotterIPN

Este repositorio contiene el firmware para la placa **NUCLEO-F334R8** cuyo objetivo es capturar dos canales analógicos mediante el periférico ADC del microcontrolador STM32F334 y enviar los valores obtenidos por la interfaz serie. Además, el código incluye un modo de simulación que genera hasta cuatro formas de onda (senoidal, cuadrada, triangular y diente de sierra) para poder probar el sistema sin requerir señales externas.

## Características principales

- **Adquisición de ADC**: se utilizan los canales `ADC1_IN1` y `ADC1_IN2` (pines `PA0` y `PA1`) en modo de conversión continua. Los datos se envían mediante `USART2` a `115200 bps` en formato `CSV`.
- **Modo de simulación**: permite transmitir señales generadas por software. El botón de usuario (`B1`) conmuta entre los datos reales y la simulación.
- **Interfaz de comunicación**: la placa utiliza `USART2` (virtual COM del ST-LINK) para transmitir los datos a un PC. `USART1` está configurado a `9600 bps` y puede emplearse para otros fines.
- **Compatibilidad**: proyecto generado con **STM32CubeIDE** (se recomienda la versión 1.8 o posterior).

## Requisitos

1. Placa **NUCLEO-F334R8**.
2. Cable micro USB para programación y comunicación (ST-LINK).
3. [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) instalado en el sistema.
4. Algún programa de terminal serie o "serial plotter" en el PC (por ejemplo, `minicom`, `PuTTY`, `CoolTerm`, etc.).

## Configuración y compilación

1. **Clonar el repositorio**

   ```bash
   git clone <URL-del-repositorio>
   cd STM32ADC_PlotterIPN
   ```
2. **Abrir el proyecto en STM32CubeIDE**

   - Ejecuta STM32CubeIDE.
   - Selecciona `File` → `Open Projects from File System...` y elige la carpeta del repositorio.
   - El IDE detectará automáticamente el archivo `.project` y cargará la configuración.
3. **Construir y programar**

   - Conecta la placa NUCLEO-F334R8 al PC mediante el puerto ST-LINK.
   - Pulsa `Project` → `Build Project` (o presiona `Ctrl+B`).
   - Una vez generado el binario, selecciona `Run` → `Debug` para programar la placa.

## Conexiones hárdware

- `PA0` (`ADC1_IN1`): primer canal analógico.
- `PA1` (`ADC1_IN2`): segundo canal analógico.
- Puerto serie por `USART2` a 115200 bps, 8N1 (virtual COM del ST-LINK).
- Botón de usuario `B1` (PC13) para alternar entre simulación y captura real.

## Uso básico

1. Abre un programa de terminal en el PC y selecciona el puerto serie asociado a la placa NUCLEO, configurado a **115200 baudios**, 8 bits de datos, sin paridad y 1 bit de parada.
2. Al iniciar la aplicación, se envían de forma periódica los valores de los canales en formato:

   ```
   valor_ch1,valor_ch2\n
   ```
3. Si se activa el modo de simulación (presionando el botón `B1`), se transmitirán hasta cuatro valores correspondientes a las señales generadas internamente:

   ```
   seno,cuadrada,triangular,sierra\n
   ```
4. Cada línea termina con `\n` para facilitar el procesamiento desde scripts en Python o desde cualquier "serial plotter".
5. Vuelve a presionar el botón `B1` para regresar a la captura de los canales analógicos reales.

## Fragmentos relevantes del código

El modo de simulación se controla mediante la variable `Simulacion` y la interrupción del botón de usuario:

```c
/* Flag que define si se envían datos simulados (true)
   o valores capturados por el ADC (false).  Se modifica
   desde la interrupción del botón de usuario. */
volatile bool Simulacion = false;
...
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == B1_Pin)
  {
    Simulacion = !Simulacion;
  }
}
```

La inicialización de `USART2` establece la velocidad de comunicación a `115200` baudios:

```c
huart2.Instance = USART2;
huart2.Init.BaudRate = 115200;
...
if (HAL_UART_Init(&huart2) != HAL_OK)
{
  Error_Handler();
}
```

## Licencia

El proyecto utiliza las bibliotecas HAL y CMSIS de STMicroelectronics, las cuales se encuentran bajo la licencia indicada en los directorios `Drivers/STM32F3xx_HAL_Driver` y `Drivers/CMSIS`.

---

Con este firmware podrás adquirir señales analógicas y visualizarlas fácilmente en tu PC o generar señales de prueba para comprobar el funcionamiento de tu cadena de procesamiento.
