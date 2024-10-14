# USB-Powered-fan Controller
This small device is designed to allow one to power and control a 12V (case)fan from a standard 5V 1A USB charger.
Intended application are small DIY fan projects such as a Desktop fan using a Noctua NV-AA1-12, Fume Extracters and Part cooling.
## Overal design:
The controller can be divided into four parts:
- A 5V->12V Boost converter based on a LMR62421XMF reference design
- A Load switch using a PMV50PEAR and a BSS138.
- A ATTiny414 that is handling PWM generation and I/O
- The I/O itself consisting of (charlieplexed) leds, Encoder, USB-C port and two 4-pin FAN pinouts
## Working
On power-up the controller will try to spin up the fan to about half speed via a 25Khz PWM signal, measure the RPM, set PWM to 80% and check if there was significant change.
If there was, then it assumes the FAN is PWM compatible and keep to using PWM, if not it will assume the FAN is DC and instead starts pulsing the Load Switch.

During operation one can set the speed of the fan anywhere between 40 and 100 percent. Pressing the encoder SW causes the controller to halt the fan till pressed again

If during operation the FAN either halts OR speeds up too much (wip), the controller will immediately shut off power and remain locked till it got power-cycled as a protection measure against sudden jamming/defects.

### TODO:
- Gerber upload
- Overhaul the code and implement a speed up detection test.
- Test with other fans.
- BOM upload
- Potentially setting up for manufacturing by PCBWay/JLCPCB
- Verify if possible to program via AVRDUDE using CH340
