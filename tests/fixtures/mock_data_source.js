// ============================
//    FAKE DATA FOR TEST 
// ============================

// Data Source, which may be from file or ROS msg. topic

const graphDOT = `digraph G { 
0 [label=I0, position_x=2, position_y=6, orientation=0.0];
1 [label=I1, position_x=2, position_y=4.8, orientation=0.0];
2 [label=I2, position_x=2, position_y=3.8, orientation=0.0];
3 [label=I3, position_x=2, position_y=2.4, orientation=0.0];
4 [label=I4, position_x=3.5, position_y=2.4, orientation=0.0];
5 [label=I5, position_x=4.8, position_y=2.4, orientation=0.0];
6 [label=I6, position_x=6, position_y=2.4, orientation=0.0];
7 [label=I7, position_x=7.2, position_y=2.4, orientation=0.0];
8 [label=I8, position_x=3.5, position_y=3.8, orientation=0.0];
9 [label=I9, position_x=4.82, position_y=3.82, orientation=0.0];
10 [label=I10, position_x=6, position_y=3.8, orientation=0.0];
11 [label=I11, position_x=7.2, position_y=3.8, orientation=0.0];
12 [label=I12, position_x=7.2, position_y=4.8, orientation=0.0];
13 [label=I13, position_x=4.8, position_y=4.8, orientation=0.0];
14 [label=I14, position_x=3.7, position_y=4.8, orientation=0.0];
16 [label=I16, position_x=3.2, position_y=1.2, orientation=0.0];
17 [label=I17, position_x=4.8, position_y=1.7, orientation=0.0];
18 [label=I18, position_x=6, position_y=1.8, orientation=0.0];
19 [label=I19, position_x=7.2, position_y=2.4, orientation=0.0];
21 [label=I21, position_x=6, position_y=4.8, orientation=0.0];
22 [label=I22, position_x=7.2, position_y=1.8, orientation=0.0];
0->1 [weight=1,width=0.0];
1->0 [weight=1,width=0.0];
1->2 [weight=1,width=0.0];
2->1 [weight=1,width=0.0];
2->3 [weight=1,width=0.0];
2->8 [weight=1,width=0.0];
3->2 [weight=1,width=0.0];
3->4 [weight=1,width=0.0];
4->3 [weight=1,width=0.0];
4->5 [weight=1,width=0.0];
4->8 [weight=1,width=0.0];
4->16 [weight=1,width=0.0];
5->4 [weight=1,width=0.0];
5->6 [weight=1,width=0.0];
5->9 [weight=1,width=0.0];
5->17 [weight=1,width=0.0];
6->5 [weight=1,width=0.0];
6->7 [weight=1,width=0.0];
6->10 [weight=1,width=0.0];
6->18 [weight=1,width=0.0];
7->6 [weight=1,width=0.0];
7->11 [weight=1,width=0.0];
7->19 [weight=1,width=0.0];
7->22 [weight=1,width=0.0];
8->2 [weight=1,width=0.0];
8->4 [weight=1,width=0.0];
8->9 [weight=1,width=0.0];
8->14 [weight=1,width=0.0];
9->5 [weight=1,width=0.0];
9->8 [weight=1,width=0.0];
9->10 [weight=1,width=0.0];
9->13 [weight=1,width=0.0];
10->6 [weight=1,width=0.0];
10->9 [weight=1,width=0.0];
10->11 [weight=1,width=0.0];
10->21 [weight=1,width=0.0];
11->7 [weight=1,width=0.0];
11->10 [weight=1,width=0.0];
11->12 [weight=1,width=0.0];
12->11 [weight=1,width=0.0];
13->9 [weight=1,width=0.0];
14->8 [weight=1,width=0.0];
16->4 [weight=1,width=0.0];
17->5 [weight=1,width=0.0];
18->6 [weight=1,width=0.0];
19->7 [weight=1,width=0.0];
21->10 [weight=1,width=0.0];
22->7 [weight=1,width=0.0];
}`;

const graphJSON = { "nodes": [{ "id": "0", "label": "I0", "x": "2", "y": "6" }, { "id": "1", "label": "I1", "x": "2", "y": "4.8" }, { "id": "2", "label": "I2", "x": "2", "y": "3.8" }, { "id": "3", "label": "I3", "x": "2", "y": "2.4" }, { "id": "4", "label": "I4", "x": "3.5", "y": "2.4" }, { "id": "5", "label": "I5", "x": "4.8", "y": "2.4" }, { "id": "6", "label": "I6", "x": "6", "y": "2.4" }, { "id": "7", "label": "I7", "x": "7.2", "y": "2.4" }, { "id": "8", "label": "I8", "x": "3.5", "y": "3.8" }, { "id": "9", "label": "I9", "x": "4.82", "y": "3.82" }, { "id": "10", "label": "I10", "x": "6", "y": "3.8" }, { "id": "11", "label": "I11", "x": "7.2", "y": "3.8" }, { "id": "12", "label": "I12", "x": "7.2", "y": "4.8" }, { "id": "13", "label": "I13", "x": "4.8", "y": "4.8" }, { "id": "14", "label": "I14", "x": "3.7", "y": "4.8" }, { "id": "16", "label": "I16", "x": "3.2", "y": "1.2" }, { "id": "17", "label": "I17", "x": "4.8", "y": "1.7" }, { "id": "18", "label": "I18", "x": "6", "y": "1.8" }, { "id": "19", "label": "I19", "x": "7.2", "y": "2.4" }, { "id": "21", "label": "I21", "x": "6", "y": "4.8" }, { "id": "22", "label": "I22", "x": "7.2", "y": "1.8" }], "edges": [{ "from": "0", "to": "1", "weight": "1", "width": "0.0" }, { "from": "1", "to": "0", "weight": "1", "width": "0.0" }, { "from": "1", "to": "2", "weight": "1", "width": "0.0" }, { "from": "2", "to": "1", "weight": "1", "width": "0.0" }, { "from": "2", "to": "3", "weight": "1", "width": "0.0" }, { "from": "2", "to": "8", "weight": "1", "width": "0.0" }, { "from": "3", "to": "2", "weight": "1", "width": "0.0" }, { "from": "3", "to": "4", "weight": "1", "width": "0.0" }, { "from": "4", "to": "3", "weight": "1", "width": "0.0" }, { "from": "4", "to": "5", "weight": "1", "width": "0.0" }, { "from": "4", "to": "8", "weight": "1", "width": "0.0" }, { "from": "4", "to": "16", "weight": "1", "width": "0.0" }, { "from": "5", "to": "4", "weight": "1", "width": "0.0" }, { "from": "5", "to": "6", "weight": "1", "width": "0.0" }, { "from": "5", "to": "9", "weight": "1", "width": "0.0" }, { "from": "5", "to": "17", "weight": "1", "width": "0.0" }, { "from": "6", "to": "5", "weight": "1", "width": "0.0" }, { "from": "6", "to": "7", "weight": "1", "width": "0.0" }, { "from": "6", "to": "10", "weight": "1", "width": "0.0" }, { "from": "6", "to": "18", "weight": "1", "width": "0.0" }, { "from": "7", "to": "6", "weight": "1", "width": "0.0" }, { "from": "7", "to": "11", "weight": "1", "width": "0.0" }, { "from": "7", "to": "19", "weight": "1", "width": "0.0" }, { "from": "7", "to": "22", "weight": "1", "width": "0.0" }, { "from": "8", "to": "2", "weight": "1", "width": "0.0" }, { "from": "8", "to": "4", "weight": "1", "width": "0.0" }, { "from": "8", "to": "9", "weight": "1", "width": "0.0" }, { "from": "8", "to": "14", "weight": "1", "width": "0.0" }, { "from": "9", "to": "5", "weight": "1", "width": "0.0" }, { "from": "9", "to": "8", "weight": "1", "width": "0.0" }, { "from": "9", "to": "10", "weight": "1", "width": "0.0" }, { "from": "9", "to": "13", "weight": "1", "width": "0.0" }, { "from": "10", "to": "6", "weight": "1", "width": "0.0" }, { "from": "10", "to": "9", "weight": "1", "width": "0.0" }, { "from": "10", "to": "11", "weight": "1", "width": "0.0" }, { "from": "10", "to": "21", "weight": "1", "width": "0.0" }, { "from": "11", "to": "7", "weight": "1", "width": "0.0" }, { "from": "11", "to": "10", "weight": "1", "width": "0.0" }, { "from": "11", "to": "12", "weight": "1", "width": "0.0" }, { "from": "12", "to": "11", "weight": "1", "width": "0.0" }, { "from": "13", "to": "9", "weight": "1", "width": "0.0" }, { "from": "14", "to": "8", "weight": "1", "width": "0.0" }, { "from": "16", "to": "4", "weight": "1", "width": "0.0" }, { "from": "17", "to": "5", "weight": "1", "width": "0.0" }, { "from": "18", "to": "6", "weight": "1", "width": "0.0" }, { "from": "19", "to": "7", "weight": "1", "width": "0.0" }, { "from": "21", "to": "10", "weight": "1", "width": "0.0" }, { "from": "22", "to": "7", "weight": "1", "width": "0.0" }] };

const graphNetwork = [{ "x": "2.0", "y": "6.0", "id": "0", "label": "I0", "connections": ["1"] }, { "x": "2.0", "y": "4.8", "id": "1", "label": "I1", "connections": ["0", "2"] }, { "x": "2.0", "y": "3.8", "id": "2", "label": "I2", "connections": ["1", "3", "8"] }, { "x": "2.0", "y": "2.4", "id": "3", "label": "I3", "connections": ["2", "4"] }, { "x": "3.5", "y": "2.4", "id": "4", "label": "I4", "connections": ["3", "5", "8", "16"] }, { "x": "4.8", "y": "2.4", "id": "5", "label": "I5", "connections": ["4", "6", "9", "17"] }, { "x": "6.0", "y": "2.4", "id": "6", "label": "I6", "connections": ["5", "7", "10", "18"] }, { "x": "7.2", "y": "2.4", "id": "7", "label": "I7", "connections": ["6", "11", "19", "22"] }, { "x": "3.5", "y": "3.8", "id": "8", "label": "I8", "connections": ["2", "4", "9", "14"] }, { "x": "4.82", "y": "3.82", "id": "9", "label": "I9", "connections": ["5", "8", "10", "13"] }, { "x": "6.0", "y": "3.8", "id": "10", "label": "I10", "connections": ["6", "9", "11", "21"] }, { "x": "7.2", "y": "3.8", "id": "11", "label": "I11", "connections": ["7", "10", "12"] }, { "x": "7.2", "y": "4.8", "id": "12", "label": "I12", "connections": ["11"] }, { "x": "4.8", "y": "4.8", "id": "13", "label": "I13", "connections": ["9"] }, { "x": "3.7", "y": "4.8", "id": "14", "label": "I14", "connections": ["8"] }, { "x": "3.2", "y": "1.2", "id": "16", "label": "I16", "connections": ["4"] }, { "x": "4.8", "y": "1.7", "id": "17", "label": "I17", "connections": ["5"] }, { "x": "6.0", "y": "1.8", "id": "18", "label": "I18", "connections": ["6"] }, { "x": "7.2", "y": "2.4", "id": "19", "label": "I19", "connections": ["7"] }, { "x": "6.0", "y": "4.8", "id": "21", "label": "I21", "connections": ["10"] }, { "x": "7.2", "y": "1.8", "id": "22", "label": "I22", "connections": ["7"] }];

const graphUniDOT = `digraph G { 
0 [label=I0, position_x=2, position_y=6, orientation=0.0];
1 [label=I1, position_x=2, position_y=4.8, orientation=0.0];
2 [label=I2, position_x=2, position_y=3.8, orientation=0.0];
3 [label=I3, position_x=2, position_y=2.4, orientation=0.0];
4 [label=I4, position_x=3.5, position_y=2.4, orientation=0.0];
5 [label=I5, position_x=4.8, position_y=2.4, orientation=0.0];
6 [label=I6, position_x=6, position_y=2.4, orientation=0.0];
7 [label=I7, position_x=7.2, position_y=2.4, orientation=0.0];
8 [label=I8, position_x=3.5, position_y=3.8, orientation=0.0];
9 [label=I9, position_x=4.8, position_y=3.8, orientation=0.0];
10 [label=I10, position_x=6, position_y=3.8, orientation=0.0];
11 [label=I11, position_x=7.2, position_y=3.8, orientation=0.0];
12 [label=I12, position_x=7.2, position_y=4.8, orientation=0.0];
13 [label=I13, position_x=4.8, position_y=4.8, orientation=0.0];
14 [label=I14, position_x=3.7, position_y=4.8, orientation=0.0];
16 [label=I16, position_x=3.2, position_y=1.2, orientation=0.0];
17 [label=I17, position_x=4.8, position_y=1.7, orientation=0.0];
18 [label=I18, position_x=6, position_y=1.8, orientation=0.0];
19 [label=I19, position_x=7.2, position_y=2.4, orientation=0.0];
21 [label=I21, position_x=6, position_y=4.8, orientation=0.0];
22 [label=I22, position_x=7.2, position_y=1.8, orientation=0.0];
1->0 [weight=1,width=0.0];
1->2 [weight=1,width=0.0];
2->1 [weight=1,width=0.0];
2->3 [weight=1,width=0.0];
3->2 [weight=1,width=0.0];
4->3 [weight=1,width=0.0];
5->4 [weight=1,width=0.0];
5->9 [weight=1,width=0.0];
5->17 [weight=1,width=0.0];
6->5 [weight=1,width=0.0];
7->6 [weight=1,width=0.0];
7->11 [weight=1,width=0.0];
7->19 [weight=1,width=0.0];
8->2 [weight=1,width=0.0];
8->14 [weight=1,width=0.0];
8->4 [weight=1,width=0.0];
9->8 [weight=1,width=0.0];
9->5 [weight=1,width=0.0];
10->9 [weight=1,width=0.0];
10->6 [weight=1,width=0.0];
10->21 [weight=1,width=0.0];
11->10 [weight=1,width=0.0];
11->7 [weight=1,width=0.0];
12->11 [weight=1,width=0.0];
13->9 [weight=1,width=0.0];
14->8 [weight=1,width=0.0];
16->4 [weight=1,width=0.0];
17->5 [weight=1,width=0.0];
18->6 [weight=1,width=0.0];
19->7 [weight=1,width=0.0];
21->10 [weight=1,width=0.0];
22->7 [weight=1,width=0.0];
}`;

const graphUniJSON = { "nodes": [{ "id": "0", "label": "I0", "x": "2", "y": "6" }, { "id": "1", "label": "I1", "x": "2", "y": "4.8" }, { "id": "2", "label": "I2", "x": "2", "y": "3.8" }, { "id": "3", "label": "I3", "x": "2", "y": "2.4" }, { "id": "4", "label": "I4", "x": "3.5", "y": "2.4" }, { "id": "5", "label": "I5", "x": "4.8", "y": "2.4" }, { "id": "6", "label": "I6", "x": "6", "y": "2.4" }, { "id": "7", "label": "I7", "x": "7.2", "y": "2.4" }, { "id": "8", "label": "I8", "x": "3.5", "y": "3.8" }, { "id": "9", "label": "I9", "x": "4.8", "y": "3.8" }, { "id": "10", "label": "I10", "x": "6", "y": "3.8" }, { "id": "11", "label": "I11", "x": "7.2", "y": "3.8" }, { "id": "12", "label": "I12", "x": "7.2", "y": "4.8" }, { "id": "13", "label": "I13", "x": "4.8", "y": "4.8" }, { "id": "14", "label": "I14", "x": "3.7", "y": "4.8" }, { "id": "16", "label": "I16", "x": "3.2", "y": "1.2" }, { "id": "17", "label": "I17", "x": "4.8", "y": "1.7" }, { "id": "18", "label": "I18", "x": "6", "y": "1.8" }, { "id": "19", "label": "I19", "x": "7.2", "y": "2.4" }, { "id": "21", "label": "I21", "x": "6", "y": "4.8" }, { "id": "22", "label": "I22", "x": "7.2", "y": "1.8" }], "edges": [{ "from": "1", "to": "0", "weight": "1", "width": "0.0" }, { "from": "1", "to": "2", "weight": "1", "width": "0.0" }, { "from": "2", "to": "1", "weight": "1", "width": "0.0" }, { "from": "2", "to": "3", "weight": "1", "width": "0.0" }, { "from": "3", "to": "2", "weight": "1", "width": "0.0" }, { "from": "4", "to": "3", "weight": "1", "width": "0.0" }, { "from": "5", "to": "4", "weight": "1", "width": "0.0" }, { "from": "5", "to": "9", "weight": "1", "width": "0.0" }, { "from": "5", "to": "17", "weight": "1", "width": "0.0" }, { "from": "6", "to": "5", "weight": "1", "width": "0.0" }, { "from": "7", "to": "6", "weight": "1", "width": "0.0" }, { "from": "7", "to": "11", "weight": "1", "width": "0.0" }, { "from": "7", "to": "19", "weight": "1", "width": "0.0" }, { "from": "8", "to": "2", "weight": "1", "width": "0.0" }, { "from": "8", "to": "14", "weight": "1", "width": "0.0" }, { "from": "8", "to": "4", "weight": "1", "width": "0.0" }, { "from": "9", "to": "8", "weight": "1", "width": "0.0" }, { "from": "9", "to": "5", "weight": "1", "width": "0.0" }, { "from": "10", "to": "9", "weight": "1", "width": "0.0" }, { "from": "10", "to": "6", "weight": "1", "width": "0.0" }, { "from": "10", "to": "21", "weight": "1", "width": "0.0" }, { "from": "11", "to": "10", "weight": "1", "width": "0.0" }, { "from": "11", "to": "7", "weight": "1", "width": "0.0" }, { "from": "12", "to": "11", "weight": "1", "width": "0.0" }, { "from": "13", "to": "9", "weight": "1", "width": "0.0" }, { "from": "14", "to": "8", "weight": "1", "width": "0.0" }, { "from": "16", "to": "4", "weight": "1", "width": "0.0" }, { "from": "17", "to": "5", "weight": "1", "width": "0.0" }, { "from": "18", "to": "6", "weight": "1", "width": "0.0" }, { "from": "19", "to": "7", "weight": "1", "width": "0.0" }, { "from": "21", "to": "10", "weight": "1", "width": "0.0" }, { "from": "22", "to": "7", "weight": "1", "width": "0.0" }] };

const graphUniNetwork = [{ "x": "2.0000", "y": "6.0000", "id": "0", "label": "I0", "connections": [] }, { "x": "2.0000", "y": "4.8000", "id": "1", "label": "I1", "connections": ["0", "2"] }, { "x": "2.0000", "y": "3.8000", "id": "2", "label": "I2", "connections": ["1", "3"] }, { "x": "2.0000", "y": "2.4000", "id": "3", "label": "I3", "connections": ["2"] }, { "x": "3.5000", "y": "2.4000", "id": "4", "label": "I4", "connections": ["3"] }, { "x": "4.8000", "y": "2.4000", "id": "5", "label": "I5", "connections": ["4", "9", "17"] }, { "x": "6.0000", "y": "2.4000", "id": "6", "label": "I6", "connections": ["5"] }, { "x": "7.2000", "y": "2.4000", "id": "7", "label": "I7", "connections": ["6", "11", "19"] }, { "x": "3.5000", "y": "3.8000", "id": "8", "label": "I8", "connections": ["2", "14", "4"] }, { "x": "4.8000", "y": "3.8000", "id": "9", "label": "I9", "connections": ["8", "5"] }, { "x": "6.0000", "y": "3.8000", "id": "10", "label": "I10", "connections": ["9", "6", "21"] }, { "x": "7.2000", "y": "3.8000", "id": "11", "label": "I11", "connections": ["10", "7"] }, { "x": "7.2000", "y": "4.8000", "id": "12", "label": "I12", "connections": ["11"] }, { "x": "4.8000", "y": "4.8000", "id": "13", "label": "I13", "connections": ["9"] }, { "x": "3.7000", "y": "4.8000", "id": "14", "label": "I14", "connections": ["8"] }, { "x": "3.2000", "y": "1.2000", "id": "16", "label": "I16", "connections": ["4"] }, { "x": "4.8000", "y": "1.7000", "id": "17", "label": "I17", "connections": ["5"] }, { "x": "6.0000", "y": "1.8000", "id": "18", "label": "I18", "connections": ["6"] }, { "x": "7.2000", "y": "2.4000", "id": "19", "label": "I19", "connections": ["7"] }, { "x": "6.0000", "y": "4.8000", "id": "21", "label": "I21", "connections": ["10"] }, { "x": "7.2000", "y": "1.8000", "id": "22", "label": "I22", "connections": ["7"] }]

module.exports = {
  graphDOT,
  graphJSON,
  graphNetwork,
  graphUniDOT,
  graphUniJSON,
  graphUniNetwork
}
