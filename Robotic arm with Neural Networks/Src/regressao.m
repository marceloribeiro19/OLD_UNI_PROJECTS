% Utilizado para calcular a reta de regressao entre as coordenadas reais e
% as coordenadas do Coppelia

x=[82.6
97.5333
110.5
123.9375
137.5
151
165.5
];

y=[-0.075
-0.05
-0.025
0
0.025
0.05
0.075
];

p=polyfit(x,y,1)

x2=[156
136
117.3667
];

y2=[0.575
0.595
0.615
];

p2=polyfit(x2,y2,1)