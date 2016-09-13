var chart = {};
var dataPoints = {};
var dataBase = {};
var paused = {};
var displaying = {};

/**
* Adds a point to a graph/plot.  Will automatically create new graph and/or plot if they do not exist
* @params chart The name of the chart you want to add the point to (must not contain spaces!)
* @params plot The name of the plot you want to add the point to (must not contain spaces!)
* @params x The x coordinate of the point
* @params y The y coordinate of the point
*/
function addPoint(chart, plot, x, y) {
	if(!dataBase[chart]) {
		dataBase[chart] = {};
		initChart(chart);
	}
	if(!dataBase[chart][plot])
		dataBase[chart][plot] = [{x:x,y:y}];
	else
		dataBase[chart][plot].push({x:x,y:y});
	if (!paused[chart]) {
		if(!dataPoints[chart])
			dataPoints[chart] = {};
		dataPoints[chart][plot] = dataBase[chart][plot].slice(dataBase[chart][plot].length-9000);
		updateChart();	
	}
}

$(function() {
    $( "#sortable" ).sortable({
    	cursor: "move",
    	axis: "y"
    });
    $( "#sortable" ).disableSelection();
    $( "#sortable" ).sortable( "disable" );
});

function initChart(chartName) {
	$("#sortable").append('<li id="'+chartName+'grid"></li>');
	$('#' + chartName + "grid").append('<div class="tivoContainer" id="'+chartName+'container"><h1><a class="dragger">&#9776;</a>'+chartName+' <a onclick="hideChart(\''+chartName+'\')" class="btn btn-danger btn-lg minButton" id="'+chartName+'minButton">&#10005;</a><a onclick="togglePause(\''+chartName+'\')" class="btn btn-primary btn-lg pauseButton" id="'+chartName+'pauseButton">&#9612;&#9612;</a></h1><div id="'+chartName+'" style="height: 300px; width: 100%;"></div><div id="'+chartName+'tivoBox" class="tivoBox"><div id="'+chartName+'tivoSlider" class="tivoSlider"></div></div></div>');
	chart[chartName] = new CanvasJS.Chart(chartName,{
		zoomEnabled: false,
		// title: {
		// 	text: "Test Graph"		
		// },
		toolTip: {
			shared: true
		},
		legend: {
			verticalAlign: "top",
			horizontalAlign: "center",
                            fontSize: 14,
			fontWeight: "light",
			fontFamily: "Source Sans Pro",
			fontColor: "dimGrey"
		},
		axisX: {
			title: "Time",
		},
		axisY:{
			prefix: '',
			includeZero: false,
			gridThickness: 1,
			gridColor: "#ddd",
		}, 
		legend:{
			cursor:"pointer",
			itemclick : function(e) {
				if (typeof(e.dataSeries.visible) === "undefined" || e.dataSeries.visible) {
					e.dataSeries.visible = false;
				} else {
					e.dataSeries.visible = true;
				}
				chart[chartName].render();
			}
		}
	});

	if(!paused[chartName]) {
		paused[chartName] = false;
	}
	else if(paused[chartName]) {
		$("#"+chartName+"pauseButton").html("<span style='font-size:20px;'>&#9658;</span>");
	}

	displaying[chartName] = true;

	$( ".dragger" ).hover(
	  function() {
	    $( "#sortable" ).sortable( "enable" );
	  }, function() {
	     $( "#sortable" ).sortable( "disable" );
	  }
	);

	$("#"+chartName+"tivoSlider").draggable({
		containment:"parent",
		drag: function(){updateChart();}
	}).resizable({
		containment:"parent",
		handles: "e,w",
		resize: function(){updateChart();}
	});
	
}

function hideChart(chartName) {
	displaying[chartName] = false;
	if(!displaying[chartName]) {
		$("#"+chartName+"container").remove();
		$("#minBar").append("<div class='minGraph' id='"+chartName+"minGraph' onclick='showChart(\""+chartName+"\")'>"+chartName+"</div>")
	}
	$("#minBar").css("background-color", "rgba(100,100,100,0.25)");
}

function showChart(chartName) {
	initChart(chartName);
	updateChart();
	$("#"+chartName+"minGraph").remove();
	if($("#minBar").html()=="") {
		$("#minBar").css("background-color", "rgba(100,100,100,0)");
	}
}

function updateChart() {
	var chartNames = Object.keys(dataBase);
	for(var j=0; j<chartNames.length; j++) {
		if(displaying[chartNames[j]]) {
			chart[chartNames[j]].options.data = [];
			var keys = Object.keys(dataBase[chartNames[j]]);
			var min = dataPoints[chartNames[j]][keys[0]][0].x;
			var max = dataPoints[chartNames[j]][keys[0]][dataPoints[chartNames[j]][keys[0]].length-1].x;
			for(i=0; i<keys.length; i++) {
				if(dataPoints[chartNames[j]][keys[i]][0].x<min)
					min = dataPoints[chartNames[j]][keys[i]][0].x;
				if(dataPoints[chartNames[j]][keys[i]][dataPoints[chartNames[j]][keys[i]].length-1].x>max)
					max = dataPoints[chartNames[j]][keys[i]][dataPoints[chartNames[j]][keys[i]].length-1].x;
				chart[chartNames[j]].options.data.push({ 
					xValueType: "dateTime",
					showInLegend: true,
					type: "line",
					name: keys[i],
					dataPoints: dataPoints[chartNames[j]][keys[i]],
					legendText: keys[i]+": "+dataPoints[chartNames[j]][keys[i]][dataPoints[chartNames[j]][keys[i]].length-1].y.toFixed(2)
				});
			}
			var x = $("#"+chartNames[j]+"tivoSlider").offset().left/$("#"+chartNames[j]+"tivoBox").width();
			chart[chartNames[j]].options.axisX.minimum = lerp(min, max, x);
			chart[chartNames[j]].options.axisX.maximum = lerp(min, max, x+($("#"+chartNames[j]+"tivoSlider").width()/$("#"+chartNames[j]+"tivoBox").width()));	
			chart[chartNames[j]].render();
		}
	}
	$(".canvasjs-chart-credit").remove();
}

function togglePause(chartName) {
	paused[chartName] = !paused[chartName];
	if(paused[chartName]) {
		$("#"+chartName+"pauseButton").html("<span style='font-size:20px;'>&#9658;</span>");
	}
	else {
		$("#"+chartName+"pauseButton").html("&#9612;&#9612;");
		var keys = Object.keys(dataBase[chartName]);
		for(i=0; i<keys.length; i++)
			dataPoints[chartName][keys[i]] = dataBase[chartName][keys[i]].slice(-9000);
	}
}

function lerp(a, b, x) {
	return a*(1-x) + b*x;
}

function initProducer() {
	addPoint("chart1", "graph1", 0, Math.random()*0.2-0.1);
	addPoint("chart1", "graph2", 0, Math.random()*0.2-0.1);
	for (var n = 0; n <= 9000; n++) {
	 	produce("chart1", "graph1");
		produce("chart1", "graph2");
	}
}

function produce(chart, plot) {
	time = dataBase[chart][plot][dataBase[chart][plot].length-1].x+1000/30;
	addPoint(chart, plot, time, dataBase[chart][plot][dataBase[chart][plot].length-1].y+Math.random()*0.2-0.1);
}
