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
"<canvas id=\"canvas\" width=\"500\" height=\"500\"",
	"style=\"background-color:#fff\">",
"</canvas>",
"<body onresize=\"resize_canvas()\">",
        "<canvas id=\"canvas\">Your browser doesn't support canvas</canvas>",
"</body>",
"<script>",
"var canvas = document.getElementById(\"canvas\");",
"canvas.setAttribute('style', \"position: absolute; top: 5%;border:2px solid blue\");",
"var ctx = canvas.getContext(\"2d\");",
"ctx.globalAlpha=1;",
"setInterval(drawClock, 100);",
"resize_canvas();",
"function degToRad(degree){",
	"var factor = Math.PI/180;",
	"return degree*factor;}",
"function drawClock() {",
	"var now = new Date();",
	"var today = now.toDateString();",
	"var time = now.toLocaleTimeString();",
	"var hours = now.getHours();",
	"var minutes = now.getMinutes();",
	"var seconds = now.getSeconds();",
	"var milisec = now.getMilliseconds();",
	"var new_sec = seconds + (milisec/1000);",
	"var radius;",
	"if(canvas.height > canvas.width)",
		"radius = canvas.width * 0.45;",
	"else",
		"radius = canvas.height * 0.45;",
	"ctx.strokeStyle = '#CfCfCf';",
	"ctx.lineWidth = 17*radius/225;",
	"ctx.shadowBlur = 25;",
	"ctx.shadowColor = '#2136F3';",
	"gradient = ctx.createRadialGradient(canvas.width/2,canvas.height/2,5,canvas.width/2,canvas.height/2,canvas.width*0.7);",
	"gradient.addColorStop(0,'#0930a3');",
	"gradient.addColorStop(1,'black');",
	"ctx.fillStyle = gradient;",
	"ctx.fillRect(0,0,canvas.width,canvas.height);",//Background
	"ctx.beginPath();",//Hours
	"ctx.arc(canvas.width/2,canvas.height/2,radius,degToRad(270), degToRad((hours*15)-89.9));",
	"ctx.stroke();",
	"ctx.beginPath();",//Minutes
	"ctx.arc(canvas.width/2,canvas.height/2,radius*0.87,degToRad(270), degToRad((minutes*6)-89.9));",
	"ctx.stroke();",
	"ctx.beginPath();",//seconds
	"ctx.arc(canvas.width/2,canvas.height/2,radius*0.74,degToRad(270), degToRad((new_sec*6)-89.9));",
	"ctx.stroke();",
	"ctx.font = getFont(radius);",
	"ctx.fillStyle = 'silver';",
	"ctx.fillText(today, canvas.width/2-85* radius/225,canvas.height/2);",
	"ctx.font = getFont(radius);",
	"ctx.fillStyle = 'silver';",
	"ctx.fillText(time, canvas.width/2-35* radius/225,canvas.height/2+30* radius/225);}",
"function getFont(radius){",
	"var size = 30* radius/225;",   // get font size based on current width
	"return (size|0) + 'px Arial';}", // set font
"function resize_canvas(){",
	"canvas = document.getElementById(\"canvas\");",
	"canvas.width  = window.innerWidth*0.4;",
	"canvas.height = window.innerHeight*0.45;}",
"</script>",
NULL
};




/*
static const char *Get_time_js[] = {
"<script> function Get_time() {"
		    "var d = new Date(); var n = d.getTime();",
		    "document.getElementById(\"rc2\").value = Math.floor(n/1000);}",
"</script>",
NULL
};*/

#endif /* JAVASCRIPT_H_ */
