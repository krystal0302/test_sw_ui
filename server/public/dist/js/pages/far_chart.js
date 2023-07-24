class DoughnutChart {
    pieCenterTextFontSize_ = pieTextFontSize * 1.5;
    pieLegendTextFontSize_ = pieTextFontSize;

    constructor(data) {
        this.element_id = data.element_id;
        this.chart_data = data.chart_data;
        this.custom_style_width = data.style_width;
        this.custom_style_height = data.style_height;
        this.sum_data_index = data.sum_data_index;
    }

    generateDoughnutChart() {
        let returnChart = new Chart(document.getElementById(this.element_id), {
            type: 'doughnut',
            data: this.chart_data,
            options: {
                // aspectRatio: 2,
                // maintainAspectRatio: false,
                cutoutPercentage: 65,
                responsive: true,
                legend: {
                    display: true,
                    position: 'right',
                    align: 'start',
                    labels: {
                        boxWidth: 14,
                        fontSize: this.pieLegendTextFontSize_,
                        fontColor: '#FFFFFF',
                        generateLabels: (chart) => {
                            const datasets = chart.data.datasets;
                            return datasets[0].data.map((data, i) => ({
                                text: `${chart.data.labels[i]}: ${data}`,
                                fillStyle: datasets[0].backgroundColor[i],
                            }))
                        }
                    }
                },
                elements: {
                    center: {
                        text: '50%',
                        color: '#FFFFFF', // Default is #000000
                        fontStyle: 'Arial', // Default is Arial
                        sidePadding: 20 // Defualt is 20 (as a percentage)
                    }
                }
            },
            plugins: [{
                beforeDraw: function (chart) {
                    // chart.canvas.parentNode.style.height = `${chart.custom_style_height}px`;
                    // chart.canvas.parentNode.style.width = `${chart.custom_style_width}px`;
                    // console.log(chart.canvas.parentNode.style.height)

                    if (chart.config.options.elements.center) {
                        //Get ctx from string
                        var ctx = chart.chart.ctx;

                        //Get options from the center object in options
                        var centerConfig = chart.config.options.elements.center;
                        var fontStyle = centerConfig.fontStyle || 'Arial';
                        var data_list = Object.values(chart.data.datasets[0].data);
                        var sum = data_list.reduce((a, b) => a + b);
                        var txt = (sum === 0) ? '0%' : Math.round((data_list[chart.sum_data_index] / sum).toFixed(2) * 100) + "%";
                        var color = centerConfig.color || '#000';
                        var sidePadding = centerConfig.sidePadding || 2000;
                        var sidePaddingCalculated = (sidePadding / 100) * (chart.innerRadius * 2)
                        // Start with a base font of 30px
                        ctx.font = "30px " + fontStyle;

                        // Get the width of the string and also the width of the element minus 10 to give it 5px side padding
                        var stringWidth = ctx.measureText(txt).width;
                        var elementWidth = (chart.innerRadius * 2) - sidePaddingCalculated;

                        // Find out how much the font can grow in width.
                        var widthRatio = elementWidth / stringWidth;
                        var newFontSize = Math.floor(25 * widthRatio);
                        var elementHeight = (chart.innerRadius * 2);

                        // Pick a new font size so it will not be larger than the height of label.
                        var fontSizeToUse = Math.min(newFontSize, elementHeight);

                        // Set font settings to draw it correctly.
                        ctx.textAlign = 'center';
                        ctx.textBaseline = 'middle';
                        var centerX = ((chart.chartArea.left + chart.chartArea.right) / 2);
                        var centerY = ((chart.chartArea.top + chart.chartArea.bottom) / 2);
                        // ctx.font = fontSizeToUse + "px " + fontStyle;
                        ctx.font = `${chart.pieCenterTextFontSize_}px ${fontStyle}`;
                        ctx.fillStyle = color;

                        // Draw text in center
                        ctx.fillText(txt, centerX, centerY);

                        // PieChart Legend
                        chart.config.options.legend.labels.fontSize = chart.pieLegendTextFontSize_;
                    }
                }
            }]
        });

        returnChart.custom_style_width = this.custom_style_width;
        returnChart.custom_style_height = this.custom_style_height;
        returnChart.pieLegendTextFontSize_ = this.pieLegendTextFontSize_;
        returnChart.pieCenterTextFontSize_ = this.pieCenterTextFontSize_;
        returnChart.sum_data_index = this.sum_data_index;

        return returnChart;
    }
}

class GanttChart {
    constructor(data) {
        this.element_id = data.element_id;
        this.is_dark_theme = data.is_dark_theme;
        this.category_list = data.category_list;
        this.legend_list = data.legend_list;
        this.scale_unit = data.scale_unit;
        this.scale_num = data.scale_num;
    }

    set startDate(_dateString) {
        this.minDate = cvtString2Date(_dateString);
    }

    get startDate() {
        return this.minDate.getTime();
    }

    set endDate(_dateString) {
        this.maxDate = cvtString2Date(_dateString);
    }

    get endDate() {
        return this.maxDate.getTime();
    }

    set chartData(_dataObjs) {
        this.chart_data = _dataObjs;
    }

    get chartData() {
        return this.chart_data;
    }

    generateGanttAMCharts() {
        var root = am5.Root.new(this.element_id);
        root.dateFormatter.setAll({
            dateFormat: "yyyy/MM/dd HH:mm:ss",
            dateFields: ["valueX", "openValueX"]
        });
        root.setThemes([
            this.is_dark_theme ? am5themes_Dark.new(root) : am5themes_Animated.new(root)
        ]);

        var chart = root.container.children.push(am5xy.XYChart.new(root, {
            wheelX: "panX",
            wheelY: "panY",
            layout: root.verticalLayout // affect the legend position
        }));

        var yAxis = chart.yAxes.push(
            am5xy.CategoryAxis.new(root, {
                categoryField: "category",
                renderer: am5xy.AxisRendererY.new(root, {}),
                tooltip: am5.Tooltip.new(root, {})
            })
        );
        yAxis.data.setAll(this.category_list);

        var xAxis = chart.xAxes.push(
            am5xy.DateAxis.new(root, {
                min: this.startDate,
                max: this.endDate,
                autoZoom: false, // keep scrollbar position even if data changed
                baseInterval: { timeUnit: this.scale_unit, count: this.scale_num },
                renderer: am5xy.AxisRendererX.new(root, {
                    minGridDistance: 140
                })
            })
        );
        xAxis.get("dateFormats")["second"] = "MMM dd HH:mm:ss";
        xAxis.get("dateFormats")["minute"] = "MMM dd HH:mm";
        xAxis.get("dateFormats")["hour"] = "MMM dd HH:mm";

        var series = chart.series.push(am5xy.ColumnSeries.new(root, {
            xAxis: xAxis,
            yAxis: yAxis,
            openValueXField: "start",
            valueXField: "end",
            categoryYField: "category"
        }));
        series.columns.template.setAll({
            templateField: "columnSettings",
            fillOpacity: 1,
            strokeOpacity: 0,
            height: am5.percent(50),
            tooltipText:
                `Task ID: {task_id}\nTask name: {task_name}\nFlow ID: {flow_id}\nFlow name: {flow_name}\nStart time: [bold]{openValueX}[/]\nEnd time: [bold]{valueX}[/]\nExecution time:{duration} s`
        });
        series.data.setAll(this.chartData);

        series.columns.template.events.on("click", function (ev) {
            let selDataCtx = ev.target.dataItem.dataContext;
            if (!selDataCtx.hasOwnProperty('flow_id')) return;
            let selFlowId = selDataCtx.flow_id;
            // highlight related tasks belong to same flow ID 
            series.columns.each(function (column) {
                if (!column.dataItem.dataContext.hasOwnProperty('flow_id')) return;
                if (column.dataItem.dataContext.flow_id === selFlowId) {
                    column.setAll({
                        fillOpacity: 1
                    });
                } else {
                    column.setAll({
                        fillOpacity: 0.3
                    });
                }
                // console.log(column._settings.fillOpacity);
            })
        });

        series.bullets.push(function () {
            var label = am5.Label.new(root, {
                text: "{task_name}",
                fill: am5.color(0x000000),
                centerY: am5.p50,
                centerX: am5.p50,
                populateText: true,
                textAlign: "center",
                oversizedBehavior: "hide"
            });

            return am5.Bullet.new(root, {
                locationX: 0.5,
                locationY: 0.5,
                sprite: label
            });
        });

        // Zoom the x-axis
        // let number = this.scale_num;
        // let unit = this.scale_unit;
        // let sDate = this.startDate;
        // let eDate = this.endDate;
        // series.events.on("datavalidated", function (ev) {
        //     let time_lag_sec = parseInt(eDate - sDate) / 1000;
        //     var add_millisecs = 0;
        //     switch (unit) {
        //         case 'second':
        //             add_millisecs = (parseInt(number) * 10) * 1000;
        //             break;
        //         case 'minute':
        //             if ((time_lag_sec / 60) < parseInt(number) * 10) return;
        //             add_millisecs = (parseInt(number) * 10) * 1000 * 60;
        //             break;
        //         case 'hour':
        //             if ((time_lag_sec / 60 / 60) < parseInt(number) * 10) return;
        //             add_millisecs = (parseInt(number) * 10) * 1000 * 60 * 60;
        //             break;
        //         default:
        //             add_millisecs = (parseInt(number) * 10) * 1000;
        //     }

        //     let dateAxis = ev.target.get("xAxis");
        //     if (dateAxis._settings.start === 0 && dateAxis._settings.end === 1) {
        //         let zoomEndDate = new Date(sDate + add_millisecs);
        //         if (dateAxis.positionToDate(dateAxis._settings.end).getTime() === zoomEndDate.getTime()) return;
        //         dateAxis.zoomToDates(new Date(sDate), zoomEndDate);
        //         // xAxis.zoomToDates(new Date(sDate), zoomEndDate);
        //     }
        // });

        // Zoom the y-axis
        yAxis.events.on("datavalidated", function (ev) {
            let catList = ev.target.data.values;
            // console.log(catList);
            if (catList.length > 4) {
                let start_cat = catList[catList.length - 4].category;
                let end_cat = catList[catList.length - 1].category;
                ev.target.zoomToCategories(start_cat, end_cat);
            }
        });

        var legend = chart.children.push(am5.Legend.new(root, {
            nameField: "name",
            fillField: "color",
            strokeField: "color",
            marginTop: 50,
            x: am5.percent(50),
            centerX: am5.percent(50),
            layout: am5.GridLayout.new(root, {
                maxColumns: 2
            })
        }));

        legend.markers.template.setAll({
            width: 30,
            height: 15
        });

        legend.data.setAll(Object.values(this.legend_list));

        chart.set("scrollbarX", am5.Scrollbar.new(root, {
            orientation: "horizontal",
            height: 30
        }));

        chart.set("scrollbarY", am5.Scrollbar.new(root, {
            orientation: "vertical",
            width: 30
        }));

        return [series, chart];
    }
}