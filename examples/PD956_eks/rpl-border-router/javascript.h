/*
 * javascript.h
 *
 *  Created on: 26/07/2018
 *      Author: pm
 */

#ifndef JAVASCRIPT_H_
#define JAVASCRIPT_H_

// Thank you Andrew Trivette: https://www.youtube.com/watch?v=9dtDaWi6R0g

static const char *clock_js[] = {
"<canvas id=\"canvas\" width=\"600\" height=\"480\"",
	"style=\"background-color:#fff\">",
"</canvas>",
"<script>",
	"var canvas=document.getElementById(\"canvas\");",
	"var ctx=canvas.getContext(\"2d\");",
	"ctx.strokeStyle='#CfCfCf';",
	"ctx.lineWidth=17;",
	"ctx.shadowBlur=25;",
	"ctx.shadowColor='#2136F3';",
	"setInterval(drawClock,100);",
	"function degToRad(degree){",
		"var factor=Math.PI/180;",
		"return degree*factor;}",
	"function drawClock(){",
		"var now=new Date();",
		"var today=now.toDateString();",
		"var time=now.toLocaleTimeString();",
		"var hours=now.getHours();",
		"var minutes=now.getMinutes();",
		"var seconds=now.getSeconds();",
		"var milisec = now.getMilliseconds();",
		"var new_sec = seconds + (milisec/1000);",
		"gradient = ctx.createRadialGradient(255,255,5,255,255,300);",
		"gradient.addColorStop(0,'#0930a3');",
		"gradient.addColorStop(1,'black');",
		"ctx.fillStyle = gradient;",  //Background
		"ctx.fillRect(0,0,canvas.width,canvas.height);",  //Hours
		"ctx.beginPath();",
		"ctx.arc(canvas.width/2,canvas.height/2,200,degToRad(270),degToRad((hours*15)-89.9));",
		"ctx.stroke();",
		"ctx.beginPath();",  //Minutes
		"ctx.arc(canvas.width/2,canvas.height/2,170,degToRad(270),degToRad((minutes*6)-89.9));",
		"ctx.stroke();",
		"ctx.beginPath();", //seconds//seconds
		"ctx.arc(canvas.width/2,canvas.height/2,140,degToRad(270),degToRad((new_sec*6)-89.9));",
		"ctx.stroke();",
		"ctx.font=\"25px Arial\";",
		"ctx.fillStyle='silver';",
		"ctx.fillText(today,canvas.width/2-85,canvas.height/2);",
		"ctx.font=\"25px Arial\";",
		"ctx.fillStyle='silver';",
		"ctx.fillText(time,canvas.width/2-35,canvas.height/2+30);}",
"</script>",
};


#endif /* JAVASCRIPT_H_ */
