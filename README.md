# STM32F4-CNC

## Serial configuration
### Pins
- PA2: USART_TX
- PA3: USART_RX
### Settings
- Baudrate: 115200
- Data bits: 8
- Stop bits: 1
- Parity: None
- Flow control: None

## Commands

| Command                                                                         | Meaning                                                                                           |
| ------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------- |
| G00 [X(number)] [Y(number)] [F(number)] G01 [X(number)] [Y(number)] [F(number)] | Absolute mode: Move in a line to (X,Y) at speed F<br>Relative Mode: Move (X,Y) amount at speed F  |
| G04 P(number                                                                    | Do nothing for P seconds                                                                          |
| G90                                                                             | absolute mode                                                                                     |
| G91                                                                             | relative mode                                                                                     | 
| G92 [X(number)] [Y(number)]                                                     | change logical position                                                                           | 
| M18                                                                             | turn off power to motors                                                                          |
| M100                                                                            | print out instructions for the human                                                              |
| M144                                                                            | report position and feedrate                                                                      |
