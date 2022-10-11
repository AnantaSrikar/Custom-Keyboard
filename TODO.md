# TODOs:
## Protocols
### USB :
- [ ] Power down from 5V to 3.3 V for microcontroller
- [ ] Implement basic USB keyboard firmware
- [ ] Figure out endpoints needed for generic USB keyboard driver
- [ ] Sending character streams to host device (macro keys)

### I2C
- [ ] Implement basic I2C enpoint to display text on SSD1306 display
- [ ] Display a static image on SSD1306
- [ ] Change image when interrupt is recieved

### Timer
- [ ] Run a timer on above chip to count down from 10

### Firmware
- [ ] Isolate keys from certain locations
- [ ] Sending character streams to host device  via USB interface (macro keys)


## Designing
### Circuitry
- [ ] Figuring out how to translate keyboard mesh to micro-controller
- [ ] Figuring out how many micro-controllers we need for the entire operation
### PCB
- [ ] Finding the optimal placement of components for maximum space utilisation
- [ ] Allowing debug pins on the circuit board to hotswap binary changes
- [ ] Handling power delivery to the entire system
- [ ] Ordering the necessary mechanical switches
### Case
- [ ] Designing an external case for the the PCB with all components
- [ ] Sheet metal cutting PCB support
- [ ] Manufacture the external of the case
- [ ] Ordering the necessary keycaps for the same