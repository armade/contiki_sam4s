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
"<script>",
"var clock_canvas = document.getElementById(\"ClockCanvas\");",
"var ctx = clock_canvas.getContext(\"2d\");",
"ctx.globalAlpha=1;",
"setInterval(drawClock, 1000);",
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
	"if(clock_canvas.height > clock_canvas.width)",
		"radius = clock_canvas.width * 0.45; ",
	"else ",
		"radius = clock_canvas.height * 0.45;",
	"ctx.strokeStyle = '#CfCfCf';",
	"ctx.lineWidth = 17*radius/225;",
	"ctx.shadowBlur = 25;",
	"ctx.shadowColor = '#2136F3';",
	"gradient = ctx.createRadialGradient(clock_canvas.width/2,clock_canvas.height/2,5,clock_canvas.width/2,clock_canvas.height/2,clock_canvas.width*0.7);",
	"gradient.addColorStop(0,'#0930a3');",
	"gradient.addColorStop(1,'black');",
	"ctx.fillStyle = gradient;",
	"ctx.fillRect(0,0,clock_canvas.width,clock_canvas.height);",//Background
	"ctx.beginPath();",//Hours
	"ctx.arc(clock_canvas.width/2,clock_canvas.height/2,radius,degToRad(270), degToRad((hours*15)-89.9));",
	"ctx.stroke();",
	"ctx.beginPath();",//Minutes
	"ctx.arc(clock_canvas.width/2,clock_canvas.height/2,radius*0.87,degToRad(270), degToRad((minutes*6)-89.9));",
	"ctx.stroke();",
	"ctx.beginPath();",//seconds
	"ctx.arc(clock_canvas.width/2,clock_canvas.height/2,radius*0.74,degToRad(270), degToRad((new_sec*6)-89.9));",
	"ctx.stroke();",
	"ctx.shadowBlur = 15;",
	"ctx.shadowColor = 'orange';",
	"ctx.font = getFont(radius);",
	"ctx.fillStyle = 'silver';",
	"ctx.fillText(today, clock_canvas.width/2-120* radius/225,clock_canvas.height/2);",
	"ctx.font = getFont(radius);",
	"ctx.fillStyle = 'silver';",
	"ctx.fillText(time, clock_canvas.width/2-95* radius/225,clock_canvas.height/2+35* radius/225);}",
"function getFont(radius){",
	"var size = 35* radius/225;",   // get font size based on current width
	"return (size|0) + 'px Arial';}", // set font
"function resize_canvas(){",
	"canvas = document.getElementById(\"clock_canvas\");",
	"canvas.width  = window.innerWidth*0.45;",
	"canvas.height = window.innerHeight*0.8;}",
"</script>",
NULL
};

static const char *snowflake_js[] = {
"<script>",
"var SnowCanvas = document.getElementById(\"SnowflakeCanvas\");",
"SnowCanvas.setAttribute('style', \"position: absolute;  left: 35px; top: 90px\");",
"var ctx_snow = SnowCanvas.getContext(\"2d\");",
"ctx_snow.strokeStyle = '#ffffff';",
"ctx_snow.lineWidth = 1.2;",
"ctx_snow.shadowBlur = 25;",
"ctx_snow.shadowColor = 'grey';",
"ctx_snow.globalAlpha=1;",
"var origo_x = SnowCanvas.width/2;",
"var origo_y = SnowCanvas.height/2;",
"var amp = SnowCanvas.width/2;",
"var amp2 = SnowCanvas.width/8;",
"var amp3 = SnowCanvas.width/16;",
"var amp4 = SnowCanvas.width/32;",
"var start_first_flake_x;",
"var start_first_flake_y;",
"var stop_first_flake_x;",
"var stop_first_flake_y;",
"var i;",
"var k;",
"for (i = 0; i < 8; i++) {",
"	ctx_snow.beginPath();",
"	ctx_snow.moveTo(origo_x,origo_y);",
"	y_line = amp*Math.sin((i*45*Math.PI/180));",
"	x_line = amp*Math.cos((i*45*Math.PI/180));",
"	ctx_snow.lineTo(origo_x+x_line,origo_y+y_line);",
"	for (k = 0; k < 4; k++) {",
"		start_first_flake_x = (amp-(amp2+k*amp3))*Math.sin((i*45*Math.PI/180));",// start point
"		start_first_flake_y = (amp-(amp2+k*amp3))*Math.cos((i*45*Math.PI/180));",
"		stop_first_flake_x = (amp-(amp4+k*amp3))*Math.sin(((i*45-(12+k*3))*Math.PI/180));",// point on line (midpoint)
"		stop_first_flake_y = (amp-(amp4+k*amp3))*Math.cos(((i*45-(12+k*3))*Math.PI/180));",
"		ctx_snow.moveTo(origo_x+stop_first_flake_x,origo_y+stop_first_flake_y);",// draw line
"		ctx_snow.lineTo(origo_x+start_first_flake_x,origo_x+start_first_flake_y);",
"		stop_first_flake_x = (amp-(amp4+k*amp3))*Math.sin(((i*45+(12+k*3))*Math.PI/180));",// Stop point
"		stop_first_flake_y = (amp-(amp4+k*amp3))*Math.cos(((i*45+(12+k*3))*Math.PI/180));",
"		ctx_snow.lineTo(origo_x+stop_first_flake_x,origo_y+stop_first_flake_y);",
"	}",
"	ctx_snow.stroke();",
"}",
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
