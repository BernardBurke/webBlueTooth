//this module takes three arguments and plots them on a chart
//the first argument is the chart id, the second is the x-axis data, and the third is the y-axis data
//the chart id is the id of the div element where the chart will be drawn
//the x-axis and y-axis data are arrays of numbers
//the function returns the chart object
// include CanvasJS library in your HTML file

function draw_chart(chart_id, x_data, y_data) {
    var chart = new CanvasJS.Chart(chart_id, {
        title: {
            text: "Chart"
        },
        axisX: {
            title: "X-axis"
        },
        axisY: {
            title: "Y-axis"
        },
        data: [{
            type: "line",
            dataPoints: []
        }]
    });

    for (var i = 0; i < x_data.length; i++) {
        chart.options.data[0].dataPoints.push({
            x: x_data[i],
            y: y_data[i]
        });
    }

    chart.render();
    return chart;
}
//write an example call to draw_chart()
//draw_chart("chartContainer", [1, 2, 3, 4], [10, 20, 30, 40]);