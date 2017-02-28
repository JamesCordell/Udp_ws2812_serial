# Udp_ws2812_serial
This project combinds the ethernt enc28j60 and the ws2812 led pixel lib to create a network for house lighting.

The master Arduino sketch will accept commands over Ethernet. This can be done using netcat. The following command will control a string of lights connected to PIN number defined in the sketch. The netcat command is sending a udp raw udb packet over the ethernet. I used a server to do this. The string is built around the following.


	"echo -n "0:showColor:0:100:100" | nc -4u -w1 192.168.0.6 5000"

1. The 0 is the id. This was so one can string multiple arduinos together via serial and respond to its own id this is a short int.

2. The "showColor" is a string that definds a command. These can be extendeed.

3. The sections after this are numbers associated with the command. In this case its red green and blue.

So far I have implimented showColour and rainbowCycle. The three numbers that accompanies this command follow the Neopixel API. 

	0:rainbowCycle:1000:10:5

the 1000 specifies a duration this is about 10 seconds. 10 is the speed as which the rainbow progresses and 5 is the nunber of colour spectrums will be squeezed onto the light strip.    
