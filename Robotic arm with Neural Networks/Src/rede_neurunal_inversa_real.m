
x = posicao_real;  % Dados de entrada
t = angles_real;   % Dados de saída

trainFcn = 'trainbr';  % Método de treino escolhido (treino Bayesiano Regularizado)
hiddenLayerSize = 30;  % Número de hidden layers
net3 = fitnet(hiddenLayerSize, trainFcn);  % Criação da rede neuronal


% Escolha da percentagem dos dados fornecidos usados para treinar, validar
% e testar a rede criada

net3.divideParam.trainRatio = 70/100;  % 70% dos dados para treino
net3.divideParam.valRatio = 15/100;    % 15% dos dados para validação
net3.divideParam.testRatio = 15/100;   % 15% dos dados para teste

[net3,tr] = train(net3,x,t); % Treino da rede

% Teste da rede

y = net3(x);  % Realiza previsões usando a rede treinada nos dados de treinamento
e = gsubtract(t, y);  % Calcula o erro entre as saídas reais e as previsões
performance = perform(net3, t, y);  % Calcula a performance da rede

