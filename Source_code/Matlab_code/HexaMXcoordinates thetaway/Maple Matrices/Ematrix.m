function [ E ] = Ematrix( x )
%EMATRIX Summary of this function goes here
%   Detailed explanation goes here
E = [(-0.231620000000000020e0 - x(1) - 0.8915e-1 * cos(x(5)) * sin(x(6))) * (((x(3) * cos(x(4)) * (-0.178300000000000014e0) + (x(2) * 0.178300000000000014e0 + 0.242808940000000077e-1) * sin(x(4))) * sin(x(5)) + (0.412978460000000061e-1 + x(1) * 0.178300000000000014e0) * cos(x(5))) * sin(x(6)) + ((-0.242808940000000077e-1 + x(2) * (-0.178300000000000014e0)) * cos(x(4)) + x(3) * sin(x(4)) * (-0.178300000000000014e0)) * cos(x(6)) + x(1) * 0.463240000000000041e0 + x(2) * 0.272360000000000047e0 + 0.801405393000000216e-1 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) (0.231620000000000020e0 - x(1) - 0.8915e-1 * cos(x(5)) * sin(x(6))) * (((x(3) * cos(x(4)) * (-0.178300000000000014e0) + (x(2) * 0.178300000000000014e0 + 0.242808940000000077e-1) * sin(x(4))) * sin(x(5)) + (-0.412978460000000061e-1 + x(1) * 0.178300000000000014e0) * cos(x(5))) * sin(x(6)) + ((-0.242808940000000077e-1 + x(2) * (-0.178300000000000014e0)) * cos(x(4)) + x(3) * sin(x(4)) * (-0.178300000000000014e0)) * cos(x(6)) + x(1) * (-0.463240000000000041e0) + x(2) * 0.272360000000000047e0 + 0.801405393000000216e-1 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) ((-0.7721e-1 * cos(x(6)) + 0.4457e-1 * sin(x(6))) * cos(x(5)) + 0.23374e0 - 0.1e1 * x(1)) * (((x(3) * cos(x(4)) * (-0.154420000000000002e0) + (x(2) * 0.154420000000000002e0 + 0.204606500000000004e-1) * sin(x(4))) * sin(x(5)) + (0.118110500000000000e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.360941307999999988e-1 + x(1) * 0.154420000000000002e0) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) - 0.118110500000000000e-1) * sin(x(4))) * sin(x(5)) + (x(2) * 0.154420000000000002e0 + 0.204606500000000004e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.154420000000000002e0 + (0.208355835999999989e-1 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + x(1) * (-0.467480000000000007e0) + x(2) * 0.265000000000000013e0 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2 + 0.801385065999999924e-1) ^ (-0.1e1 / 0.2e1) ((-0.7721e-1 * cos(x(6)) + 0.4457e-1 * sin(x(6))) * cos(x(5)) + 0.213e-2 - 0.1e1 * x(1)) * (((x(3) * cos(x(4)) * (-0.154420000000000002e0) + (x(2) * 0.154420000000000002e0 - 0.414880214000000053e-1) * sin(x(4))) * sin(x(5)) + (-0.239492438000000023e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.328914600000000017e-3 + x(1) * 0.154420000000000002e0) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) + 0.239492438000000023e-1) * sin(x(4))) * sin(x(5)) + (x(2) * 0.154420000000000002e0 - 0.414880214000000053e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.154420000000000002e0 + (0.189868199999999979e-3 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + 0.801359748000000094e-1 + x(1) * (-0.425999999999999990e-2) + x(2) * (-0.537340000000000040e0) + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) ((0.7721e-1 * cos(x(6)) + 0.4457e-1 * sin(x(6))) * cos(x(5)) - 0.213e-2 - 0.1e1 * x(1)) * (((x(3) * cos(x(4)) * 0.154420000000000002e0 + (x(2) * (-0.154420000000000002e0) + 0.414880214000000053e-1) * sin(x(4))) * sin(x(5)) + (-0.239492438000000023e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.328914600000000017e-3 + x(1) * (-0.154420000000000002e0)) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) + 0.239492438000000023e-1) * sin(x(4))) * sin(x(5)) + (x(2) * (-0.154420000000000002e0) + 0.414880214000000053e-1) * cos(x(4)) + x(3) * sin(x(4)) * (-0.154420000000000002e0) + (-0.189868199999999979e-3 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + 0.801359748000000094e-1 + x(1) * 0.425999999999999990e-2 + x(2) * (-0.537340000000000040e0) + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) ((0.7721e-1 * cos(x(6)) + 0.4457e-1 * sin(x(6))) * cos(x(5)) - 0.23374e0 - 0.1e1 * x(1)) * (((x(3) * cos(x(4)) * 0.154420000000000002e0 + (x(2) * (-0.154420000000000002e0) - 0.204606500000000004e-1) * sin(x(4))) * sin(x(5)) + (0.118110500000000000e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.360941307999999988e-1 + x(1) * (-0.154420000000000002e0)) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) - 0.118110500000000000e-1) * sin(x(4))) * sin(x(5)) + (x(2) * (-0.154420000000000002e0) - 0.204606500000000004e-1) * cos(x(4)) + x(3) * sin(x(4)) * (-0.154420000000000002e0) + (-0.208355835999999989e-1 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + x(1) * 0.467480000000000007e0 + x(2) * 0.265000000000000013e0 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2 + 0.801385065999999924e-1) ^ (-0.1e1 / 0.2e1); (((x(3) * cos(x(4)) * (-0.178300000000000014e0) + (x(2) * 0.178300000000000014e0 + 0.242808940000000077e-1) * sin(x(4))) * sin(x(5)) + (0.412978460000000061e-1 + x(1) * 0.178300000000000014e0) * cos(x(5))) * sin(x(6)) + ((-0.242808940000000077e-1 + x(2) * (-0.178300000000000014e0)) * cos(x(4)) + x(3) * sin(x(4)) * (-0.178300000000000014e0)) * cos(x(6)) + x(1) * 0.463240000000000041e0 + x(2) * 0.272360000000000047e0 + 0.801405393000000216e-1 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) * (-0.136180000000000023e0 - x(2) - 0.8915e-1 * sin(x(4)) * sin(x(5)) * sin(x(6)) + 0.8915e-1 * cos(x(4)) * cos(x(6))) (((x(3) * cos(x(4)) * (-0.178300000000000014e0) + (x(2) * 0.178300000000000014e0 + 0.242808940000000077e-1) * sin(x(4))) * sin(x(5)) + (-0.412978460000000061e-1 + x(1) * 0.178300000000000014e0) * cos(x(5))) * sin(x(6)) + ((-0.242808940000000077e-1 + x(2) * (-0.178300000000000014e0)) * cos(x(4)) + x(3) * sin(x(4)) * (-0.178300000000000014e0)) * cos(x(6)) + x(1) * (-0.463240000000000041e0) + x(2) * 0.272360000000000047e0 + 0.801405393000000216e-1 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) * (-0.136180000000000023e0 - x(2) - 0.8915e-1 * sin(x(4)) * sin(x(5)) * sin(x(6)) + 0.8915e-1 * cos(x(4)) * cos(x(6))) ((-0.7721e-1 * sin(x(6)) - 0.4457e-1 * cos(x(6))) * cos(x(4)) + (-0.7721e-1 * cos(x(6)) + 0.4457e-1 * sin(x(6))) * sin(x(5)) * sin(x(4)) - 0.1325e0 - 0.1e1 * x(2)) * (((x(3) * cos(x(4)) * (-0.154420000000000002e0) + (x(2) * 0.154420000000000002e0 + 0.204606500000000004e-1) * sin(x(4))) * sin(x(5)) + (0.118110500000000000e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.360941307999999988e-1 + x(1) * 0.154420000000000002e0) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) - 0.118110500000000000e-1) * sin(x(4))) * sin(x(5)) + (x(2) * 0.154420000000000002e0 + 0.204606500000000004e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.154420000000000002e0 + (0.208355835999999989e-1 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + x(1) * (-0.467480000000000007e0) + x(2) * 0.265000000000000013e0 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2 + 0.801385065999999924e-1) ^ (-0.1e1 / 0.2e1) ((-0.7721e-1 * sin(x(6)) - 0.4457e-1 * cos(x(6))) * cos(x(4)) + (-0.7721e-1 * cos(x(6)) + 0.4457e-1 * sin(x(6))) * sin(x(5)) * sin(x(4)) + 0.26867e0 - 0.1e1 * x(2)) * (((x(3) * cos(x(4)) * (-0.154420000000000002e0) + (x(2) * 0.154420000000000002e0 - 0.414880214000000053e-1) * sin(x(4))) * sin(x(5)) + (-0.239492438000000023e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.328914600000000017e-3 + x(1) * 0.154420000000000002e0) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) + 0.239492438000000023e-1) * sin(x(4))) * sin(x(5)) + (x(2) * 0.154420000000000002e0 - 0.414880214000000053e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.154420000000000002e0 + (0.189868199999999979e-3 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + 0.801359748000000094e-1 + x(1) * (-0.425999999999999990e-2) + x(2) * (-0.537340000000000040e0) + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) ((0.7721e-1 * sin(x(6)) - 0.4457e-1 * cos(x(6))) * cos(x(4)) + (0.7721e-1 * cos(x(6)) + 0.4457e-1 * sin(x(6))) * sin(x(5)) * sin(x(4)) + 0.26867e0 - 0.1e1 * x(2)) * (((x(3) * cos(x(4)) * 0.154420000000000002e0 + (x(2) * (-0.154420000000000002e0) + 0.414880214000000053e-1) * sin(x(4))) * sin(x(5)) + (-0.239492438000000023e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.328914600000000017e-3 + x(1) * (-0.154420000000000002e0)) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) + 0.239492438000000023e-1) * sin(x(4))) * sin(x(5)) + (x(2) * (-0.154420000000000002e0) + 0.414880214000000053e-1) * cos(x(4)) + x(3) * sin(x(4)) * (-0.154420000000000002e0) + (-0.189868199999999979e-3 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + 0.801359748000000094e-1 + x(1) * 0.425999999999999990e-2 + x(2) * (-0.537340000000000040e0) + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) ((0.7721e-1 * sin(x(6)) - 0.4457e-1 * cos(x(6))) * cos(x(4)) + (0.7721e-1 * cos(x(6)) + 0.4457e-1 * sin(x(6))) * sin(x(5)) * sin(x(4)) - 0.1325e0 - 0.1e1 * x(2)) * (((x(3) * cos(x(4)) * 0.154420000000000002e0 + (x(2) * (-0.154420000000000002e0) - 0.204606500000000004e-1) * sin(x(4))) * sin(x(5)) + (0.118110500000000000e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.360941307999999988e-1 + x(1) * (-0.154420000000000002e0)) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) - 0.118110500000000000e-1) * sin(x(4))) * sin(x(5)) + (x(2) * (-0.154420000000000002e0) - 0.204606500000000004e-1) * cos(x(4)) + x(3) * sin(x(4)) * (-0.154420000000000002e0) + (-0.208355835999999989e-1 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + x(1) * 0.467480000000000007e0 + x(2) * 0.265000000000000013e0 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2 + 0.801385065999999924e-1) ^ (-0.1e1 / 0.2e1); (((x(3) * cos(x(4)) * (-0.178300000000000014e0) + (x(2) * 0.178300000000000014e0 + 0.242808940000000077e-1) * sin(x(4))) * sin(x(5)) + (0.412978460000000061e-1 + x(1) * 0.178300000000000014e0) * cos(x(5))) * sin(x(6)) + ((-0.242808940000000077e-1 + x(2) * (-0.178300000000000014e0)) * cos(x(4)) + x(3) * sin(x(4)) * (-0.178300000000000014e0)) * cos(x(6)) + x(1) * 0.463240000000000041e0 + x(2) * 0.272360000000000047e0 + 0.801405393000000216e-1 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) * (-x(3) + 0.8915e-1 * cos(x(4)) * sin(x(5)) * sin(x(6)) + 0.8915e-1 * sin(x(4)) * cos(x(6))) (((x(3) * cos(x(4)) * (-0.178300000000000014e0) + (x(2) * 0.178300000000000014e0 + 0.242808940000000077e-1) * sin(x(4))) * sin(x(5)) + (-0.412978460000000061e-1 + x(1) * 0.178300000000000014e0) * cos(x(5))) * sin(x(6)) + ((-0.242808940000000077e-1 + x(2) * (-0.178300000000000014e0)) * cos(x(4)) + x(3) * sin(x(4)) * (-0.178300000000000014e0)) * cos(x(6)) + x(1) * (-0.463240000000000041e0) + x(2) * 0.272360000000000047e0 + 0.801405393000000216e-1 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) * (-x(3) + 0.8915e-1 * cos(x(4)) * sin(x(5)) * sin(x(6)) + 0.8915e-1 * sin(x(4)) * cos(x(6))) ((0.7721e-1 * cos(x(6)) - 0.4457e-1 * sin(x(6))) * sin(x(5)) * cos(x(4)) + (-0.7721e-1 * sin(x(6)) - 0.4457e-1 * cos(x(6))) * sin(x(4)) - 0.1e1 * x(3)) * (((x(3) * cos(x(4)) * (-0.154420000000000002e0) + (x(2) * 0.154420000000000002e0 + 0.204606500000000004e-1) * sin(x(4))) * sin(x(5)) + (0.118110500000000000e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.360941307999999988e-1 + x(1) * 0.154420000000000002e0) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) - 0.118110500000000000e-1) * sin(x(4))) * sin(x(5)) + (x(2) * 0.154420000000000002e0 + 0.204606500000000004e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.154420000000000002e0 + (0.208355835999999989e-1 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + x(1) * (-0.467480000000000007e0) + x(2) * 0.265000000000000013e0 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2 + 0.801385065999999924e-1) ^ (-0.1e1 / 0.2e1) ((0.7721e-1 * cos(x(6)) - 0.4457e-1 * sin(x(6))) * sin(x(5)) * cos(x(4)) + (-0.7721e-1 * sin(x(6)) - 0.4457e-1 * cos(x(6))) * sin(x(4)) - 0.1e1 * x(3)) * (((x(3) * cos(x(4)) * (-0.154420000000000002e0) + (x(2) * 0.154420000000000002e0 - 0.414880214000000053e-1) * sin(x(4))) * sin(x(5)) + (-0.239492438000000023e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.328914600000000017e-3 + x(1) * 0.154420000000000002e0) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) + 0.239492438000000023e-1) * sin(x(4))) * sin(x(5)) + (x(2) * 0.154420000000000002e0 - 0.414880214000000053e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.154420000000000002e0 + (0.189868199999999979e-3 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + 0.801359748000000094e-1 + x(1) * (-0.425999999999999990e-2) + x(2) * (-0.537340000000000040e0) + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) ((-0.7721e-1 * cos(x(6)) - 0.4457e-1 * sin(x(6))) * sin(x(5)) * cos(x(4)) + (0.7721e-1 * sin(x(6)) - 0.4457e-1 * cos(x(6))) * sin(x(4)) - 0.1e1 * x(3)) * (((x(3) * cos(x(4)) * 0.154420000000000002e0 + (x(2) * (-0.154420000000000002e0) + 0.414880214000000053e-1) * sin(x(4))) * sin(x(5)) + (-0.239492438000000023e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.328914600000000017e-3 + x(1) * (-0.154420000000000002e0)) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) + 0.239492438000000023e-1) * sin(x(4))) * sin(x(5)) + (x(2) * (-0.154420000000000002e0) + 0.414880214000000053e-1) * cos(x(4)) + x(3) * sin(x(4)) * (-0.154420000000000002e0) + (-0.189868199999999979e-3 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + 0.801359748000000094e-1 + x(1) * 0.425999999999999990e-2 + x(2) * (-0.537340000000000040e0) + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) ((-0.7721e-1 * cos(x(6)) - 0.4457e-1 * sin(x(6))) * sin(x(5)) * cos(x(4)) + (0.7721e-1 * sin(x(6)) - 0.4457e-1 * cos(x(6))) * sin(x(4)) - 0.1e1 * x(3)) * (((x(3) * cos(x(4)) * 0.154420000000000002e0 + (x(2) * (-0.154420000000000002e0) - 0.204606500000000004e-1) * sin(x(4))) * sin(x(5)) + (0.118110500000000000e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.360941307999999988e-1 + x(1) * (-0.154420000000000002e0)) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) - 0.118110500000000000e-1) * sin(x(4))) * sin(x(5)) + (x(2) * (-0.154420000000000002e0) - 0.204606500000000004e-1) * cos(x(4)) + x(3) * sin(x(4)) * (-0.154420000000000002e0) + (-0.208355835999999989e-1 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + x(1) * 0.467480000000000007e0 + x(2) * 0.265000000000000013e0 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2 + 0.801385065999999924e-1) ^ (-0.1e1 / 0.2e1); (((-0.121404470000000039e-1 - 0.8915e-1 * x(2)) * sin(x(6)) * sin(x(5)) + 0.8915e-1 * x(3) * cos(x(6))) * cos(x(4)) + (-0.8915e-1 * sin(x(5)) * sin(x(6)) * x(3) + (-0.121404470000000039e-1 - 0.8915e-1 * x(2)) * cos(x(6))) * sin(x(4))) * (((x(3) * cos(x(4)) * (-0.178300000000000014e0) + (x(2) * 0.178300000000000014e0 + 0.242808940000000077e-1) * sin(x(4))) * sin(x(5)) + (0.412978460000000061e-1 + x(1) * 0.178300000000000014e0) * cos(x(5))) * sin(x(6)) + ((-0.242808940000000077e-1 + x(2) * (-0.178300000000000014e0)) * cos(x(4)) + x(3) * sin(x(4)) * (-0.178300000000000014e0)) * cos(x(6)) + x(1) * 0.463240000000000041e0 + x(2) * 0.272360000000000047e0 + 0.801405393000000216e-1 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) (((-0.121404470000000039e-1 - 0.8915e-1 * x(2)) * sin(x(6)) * sin(x(5)) + 0.8915e-1 * x(3) * cos(x(6))) * cos(x(4)) + (-0.8915e-1 * sin(x(5)) * sin(x(6)) * x(3) + (-0.121404470000000039e-1 - 0.8915e-1 * x(2)) * cos(x(6))) * sin(x(4))) * (((x(3) * cos(x(4)) * (-0.178300000000000014e0) + (x(2) * 0.178300000000000014e0 + 0.242808940000000077e-1) * sin(x(4))) * sin(x(5)) + (-0.412978460000000061e-1 + x(1) * 0.178300000000000014e0) * cos(x(5))) * sin(x(6)) + ((-0.242808940000000077e-1 + x(2) * (-0.178300000000000014e0)) * cos(x(4)) + x(3) * sin(x(4)) * (-0.178300000000000014e0)) * cos(x(6)) + x(1) * (-0.463240000000000041e0) + x(2) * 0.272360000000000047e0 + 0.801405393000000216e-1 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) ((((-0.10230325e-1 - 0.7721e-1 * x(2)) * sin(x(5)) - 0.4457e-1 * x(3)) * cos(x(4)) + (0.5905525e-2 + 0.4457e-1 * x(2) - 0.7721e-1 * x(3) * sin(x(5))) * sin(x(4))) * cos(x(6)) + ((0.5905525e-2 + 0.4457e-1 * x(2)) * sin(x(5)) - 0.7721e-1 * x(3)) * sin(x(6)) * cos(x(4)) + (0.10230325e-1 + 0.7721e-1 * x(2) + 0.4457e-1 * x(3) * sin(x(5))) * sin(x(4)) * sin(x(6))) * (((x(3) * cos(x(4)) * (-0.154420000000000002e0) + (x(2) * 0.154420000000000002e0 + 0.204606500000000004e-1) * sin(x(4))) * sin(x(5)) + (0.118110500000000000e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.360941307999999988e-1 + x(1) * 0.154420000000000002e0) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) - 0.118110500000000000e-1) * sin(x(4))) * sin(x(5)) + (x(2) * 0.154420000000000002e0 + 0.204606500000000004e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.154420000000000002e0 + (0.208355835999999989e-1 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + x(1) * (-0.467480000000000007e0) + x(2) * 0.265000000000000013e0 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2 + 0.801385065999999924e-1) ^ (-0.1e1 / 0.2e1) ((((0.207440107000000026e-1 - 0.7721e-1 * x(2)) * sin(x(5)) - 0.4457e-1 * x(3)) * cos(x(4)) + (-0.7721e-1 * x(3) * sin(x(5)) + 0.4457e-1 * x(2) - 0.119746219000000011e-1) * sin(x(4))) * cos(x(6)) + ((-0.119746219000000011e-1 + 0.4457e-1 * x(2)) * sin(x(5)) - 0.7721e-1 * x(3)) * sin(x(6)) * cos(x(4)) + (0.4457e-1 * x(3) * sin(x(5)) + 0.7721e-1 * x(2) - 0.207440107000000026e-1) * sin(x(4)) * sin(x(6))) * (((x(3) * cos(x(4)) * (-0.154420000000000002e0) + (x(2) * 0.154420000000000002e0 - 0.414880214000000053e-1) * sin(x(4))) * sin(x(5)) + (-0.239492438000000023e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.328914600000000017e-3 + x(1) * 0.154420000000000002e0) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) + 0.239492438000000023e-1) * sin(x(4))) * sin(x(5)) + (x(2) * 0.154420000000000002e0 - 0.414880214000000053e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.154420000000000002e0 + (0.189868199999999979e-3 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + 0.801359748000000094e-1 + x(1) * (-0.425999999999999990e-2) + x(2) * (-0.537340000000000040e0) + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) ((((-0.119746219009483918e-1 + 0.4457000000e-1 * x(2)) * sin(x(5)) + 0.7721000002e-1 * x(3)) * sin(x(6)) + (-0.207440107016429280e-1 + 0.7721000002e-1 * x(2)) * cos(x(6)) * sin(x(5)) - 0.4457000000e-1 * x(3) * cos(x(6))) * cos(x(4)) + ((0.4457000000e-1 * x(3) * sin(x(5)) - 0.7721000002e-1 * x(2) + 0.207440107016429280e-1) * sin(x(6)) + 0.7721000002e-1 * sin(x(5)) * cos(x(6)) * x(3) + (-0.119746219009483918e-1 + 0.4457000000e-1 * x(2)) * cos(x(6))) * sin(x(4))) * (((x(3) * cos(x(4)) * 0.154420000000000002e0 + (x(2) * (-0.154420000000000002e0) + 0.414880214000000053e-1) * sin(x(4))) * sin(x(5)) + (-0.239492438000000023e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.328914600000000017e-3 + x(1) * (-0.154420000000000002e0)) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) + 0.239492438000000023e-1) * sin(x(4))) * sin(x(5)) + (x(2) * (-0.154420000000000002e0) + 0.414880214000000053e-1) * cos(x(4)) + x(3) * sin(x(4)) * (-0.154420000000000002e0) + (-0.189868199999999979e-3 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + 0.801359748000000094e-1 + x(1) * 0.425999999999999990e-2 + x(2) * (-0.537340000000000040e0) + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) ((((0.7721e-1 * x(2) + 0.10230325e-1) * sin(x(5)) - 0.4457e-1 * x(3)) * cos(x(4)) + (0.7721e-1 * x(3) * sin(x(5)) + 0.5905525e-2 + 0.4457e-1 * x(2)) * sin(x(4))) * cos(x(6)) + ((0.5905525e-2 + 0.4457e-1 * x(2)) * sin(x(5)) + 0.7721e-1 * x(3)) * sin(x(6)) * cos(x(4)) + (0.4457e-1 * x(3) * sin(x(5)) - 0.10230325e-1 - 0.7721e-1 * x(2)) * sin(x(4)) * sin(x(6))) * (((x(3) * cos(x(4)) * 0.154420000000000002e0 + (x(2) * (-0.154420000000000002e0) - 0.204606500000000004e-1) * sin(x(4))) * sin(x(5)) + (0.118110500000000000e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.360941307999999988e-1 + x(1) * (-0.154420000000000002e0)) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) - 0.118110500000000000e-1) * sin(x(4))) * sin(x(5)) + (x(2) * (-0.154420000000000002e0) - 0.204606500000000004e-1) * cos(x(4)) + x(3) * sin(x(4)) * (-0.154420000000000002e0) + (-0.208355835999999989e-1 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + x(1) * 0.467480000000000007e0 + x(2) * 0.265000000000000013e0 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2 + 0.801385065999999924e-1) ^ (-0.1e1 / 0.2e1); (0.8915e-1 * cos(x(4)) * cos(x(5)) * x(3) + (-0.121404470044676870e-1 - 0.8915e-1 * x(2)) * cos(x(5)) * sin(x(4)) + (0.206489230075988031e-1 + 0.8915e-1 * x(1)) * sin(x(5))) * sin(x(6)) * (((x(3) * cos(x(4)) * (-0.178300000000000014e0) + (x(2) * 0.178300000000000014e0 + 0.242808940000000077e-1) * sin(x(4))) * sin(x(5)) + (0.412978460000000061e-1 + x(1) * 0.178300000000000014e0) * cos(x(5))) * sin(x(6)) + ((-0.242808940000000077e-1 + x(2) * (-0.178300000000000014e0)) * cos(x(4)) + x(3) * sin(x(4)) * (-0.178300000000000014e0)) * cos(x(6)) + x(1) * 0.463240000000000041e0 + x(2) * 0.272360000000000047e0 + 0.801405393000000216e-1 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) (0.8915e-1 * cos(x(4)) * cos(x(5)) * x(3) + (-0.121404470044676870e-1 - 0.8915e-1 * x(2)) * cos(x(5)) * sin(x(4)) + (-0.206489230075988031e-1 + 0.8915e-1 * x(1)) * sin(x(5))) * sin(x(6)) * (((x(3) * cos(x(4)) * (-0.178300000000000014e0) + (x(2) * 0.178300000000000014e0 + 0.242808940000000077e-1) * sin(x(4))) * sin(x(5)) + (-0.412978460000000061e-1 + x(1) * 0.178300000000000014e0) * cos(x(5))) * sin(x(6)) + ((-0.242808940000000077e-1 + x(2) * (-0.178300000000000014e0)) * cos(x(4)) + x(3) * sin(x(4)) * (-0.178300000000000014e0)) * cos(x(6)) + x(1) * (-0.463240000000000041e0) + x(2) * 0.272360000000000047e0 + 0.801405393000000216e-1 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) ((((-0.10230325e-1 - 0.7721e-1 * x(2)) * cos(x(6)) + (0.5905525e-2 + 0.4457e-1 * x(2)) * sin(x(6))) * sin(x(4)) + 0.7721e-1 * cos(x(4)) * cos(x(6)) * x(3) - 0.4457e-1 * cos(x(4)) * sin(x(6)) * x(3)) * cos(x(5)) + ((-0.180470654e-1 + 0.7721e-1 * x(1)) * cos(x(6)) + (0.104177918e-1 - 0.4457e-1 * x(1)) * sin(x(6))) * sin(x(5))) * (((x(3) * cos(x(4)) * (-0.154420000000000002e0) + (x(2) * 0.154420000000000002e0 + 0.204606500000000004e-1) * sin(x(4))) * sin(x(5)) + (0.118110500000000000e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.360941307999999988e-1 + x(1) * 0.154420000000000002e0) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) - 0.118110500000000000e-1) * sin(x(4))) * sin(x(5)) + (x(2) * 0.154420000000000002e0 + 0.204606500000000004e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.154420000000000002e0 + (0.208355835999999989e-1 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + x(1) * (-0.467480000000000007e0) + x(2) * 0.265000000000000013e0 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2 + 0.801385065999999924e-1) ^ (-0.1e1 / 0.2e1) ((0.7721000002e-1 * x(3) * cos(x(6)) - 0.4457000000e-1 * x(3) * sin(x(6))) * cos(x(5)) * cos(x(4)) + ((-0.7721000002e-1 * x(2) + 0.207440107016429280e-1) * cos(x(6)) + (-0.119746219009483918e-1 + 0.4457000000e-1 * x(2)) * sin(x(6))) * cos(x(5)) * sin(x(4)) + ((-0.1644573000e-3 + 0.7721000002e-1 * x(1)) * cos(x(6)) + (0.949341000075187689e-4 - 0.4457000000e-1 * x(1)) * sin(x(6))) * sin(x(5))) * (((x(3) * cos(x(4)) * (-0.154420000000000002e0) + (x(2) * 0.154420000000000002e0 - 0.414880214000000053e-1) * sin(x(4))) * sin(x(5)) + (-0.239492438000000023e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.328914600000000017e-3 + x(1) * 0.154420000000000002e0) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) + 0.239492438000000023e-1) * sin(x(4))) * sin(x(5)) + (x(2) * 0.154420000000000002e0 - 0.414880214000000053e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.154420000000000002e0 + (0.189868199999999979e-3 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + 0.801359748000000094e-1 + x(1) * (-0.425999999999999990e-2) + x(2) * (-0.537340000000000040e0) + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) ((-0.7721000002e-1 * x(3) * cos(x(6)) - 0.4457000000e-1 * x(3) * sin(x(6))) * cos(x(5)) * cos(x(4)) + ((-0.207440107016429280e-1 + 0.7721000002e-1 * x(2)) * cos(x(6)) + (-0.119746219009483918e-1 + 0.4457000000e-1 * x(2)) * sin(x(6))) * cos(x(5)) * sin(x(4)) + ((-0.1644573000e-3 - 0.7721000002e-1 * x(1)) * cos(x(6)) + (-0.949341000075187689e-4 - 0.4457000000e-1 * x(1)) * sin(x(6))) * sin(x(5))) * (((x(3) * cos(x(4)) * 0.154420000000000002e0 + (x(2) * (-0.154420000000000002e0) + 0.414880214000000053e-1) * sin(x(4))) * sin(x(5)) + (-0.239492438000000023e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.328914600000000017e-3 + x(1) * (-0.154420000000000002e0)) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) + 0.239492438000000023e-1) * sin(x(4))) * sin(x(5)) + (x(2) * (-0.154420000000000002e0) + 0.414880214000000053e-1) * cos(x(4)) + x(3) * sin(x(4)) * (-0.154420000000000002e0) + (-0.189868199999999979e-3 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + 0.801359748000000094e-1 + x(1) * 0.425999999999999990e-2 + x(2) * (-0.537340000000000040e0) + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) ((((0.7721e-1 * x(2) + 0.10230325e-1) * cos(x(6)) + (0.5905525e-2 + 0.4457e-1 * x(2)) * sin(x(6))) * sin(x(4)) - 0.7721e-1 * cos(x(4)) * cos(x(6)) * x(3) - 0.4457e-1 * cos(x(4)) * sin(x(6)) * x(3)) * cos(x(5)) + ((-0.180470654e-1 - 0.7721e-1 * x(1)) * cos(x(6)) + (-0.104177918e-1 - 0.4457e-1 * x(1)) * sin(x(6))) * sin(x(5))) * (((x(3) * cos(x(4)) * 0.154420000000000002e0 + (x(2) * (-0.154420000000000002e0) - 0.204606500000000004e-1) * sin(x(4))) * sin(x(5)) + (0.118110500000000000e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.360941307999999988e-1 + x(1) * (-0.154420000000000002e0)) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) - 0.118110500000000000e-1) * sin(x(4))) * sin(x(5)) + (x(2) * (-0.154420000000000002e0) - 0.204606500000000004e-1) * cos(x(4)) + x(3) * sin(x(4)) * (-0.154420000000000002e0) + (-0.208355835999999989e-1 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + x(1) * 0.467480000000000007e0 + x(2) * 0.265000000000000013e0 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2 + 0.801385065999999924e-1) ^ (-0.1e1 / 0.2e1); ((0.8915e-1 * x(3) * cos(x(4)) * sin(x(5)) + (-0.121404470000000039e-1 - 0.8915e-1 * x(2)) * sin(x(5)) * sin(x(4)) + (-0.206489230000000031e-1 - 0.8915e-1 * x(1)) * cos(x(5))) * cos(x(6)) + (-0.121404470000000039e-1 - 0.8915e-1 * x(2)) * sin(x(6)) * cos(x(4)) - 0.8915e-1 * x(3) * sin(x(4)) * sin(x(6))) * (((x(3) * cos(x(4)) * (-0.178300000000000014e0) + (x(2) * 0.178300000000000014e0 + 0.242808940000000077e-1) * sin(x(4))) * sin(x(5)) + (0.412978460000000061e-1 + x(1) * 0.178300000000000014e0) * cos(x(5))) * sin(x(6)) + ((-0.242808940000000077e-1 + x(2) * (-0.178300000000000014e0)) * cos(x(4)) + x(3) * sin(x(4)) * (-0.178300000000000014e0)) * cos(x(6)) + x(1) * 0.463240000000000041e0 + x(2) * 0.272360000000000047e0 + 0.801405393000000216e-1 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) ((0.8915e-1 * x(3) * cos(x(4)) * sin(x(5)) + (-0.121404470000000039e-1 - 0.8915e-1 * x(2)) * sin(x(5)) * sin(x(4)) + (0.206489230000000031e-1 - 0.8915e-1 * x(1)) * cos(x(5))) * cos(x(6)) + (-0.121404470000000039e-1 - 0.8915e-1 * x(2)) * sin(x(6)) * cos(x(4)) - 0.8915e-1 * x(3) * sin(x(4)) * sin(x(6))) * (((x(3) * cos(x(4)) * (-0.178300000000000014e0) + (x(2) * 0.178300000000000014e0 + 0.242808940000000077e-1) * sin(x(4))) * sin(x(5)) + (-0.412978460000000061e-1 + x(1) * 0.178300000000000014e0) * cos(x(5))) * sin(x(6)) + ((-0.242808940000000077e-1 + x(2) * (-0.178300000000000014e0)) * cos(x(4)) + x(3) * sin(x(4)) * (-0.178300000000000014e0)) * cos(x(6)) + x(1) * (-0.463240000000000041e0) + x(2) * 0.272360000000000047e0 + 0.801405393000000216e-1 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) (((-0.4457000001e-1 * x(3) * cos(x(4)) + (0.5905525002e-2 + 0.4457000001e-1 * x(2)) * sin(x(4))) * sin(x(5)) + (-0.7721000000e-1 * x(2) - 0.1023032500e-1) * cos(x(4)) - 0.7721000000e-1 * x(3) * sin(x(4)) + (-0.1041779181e-1 + 0.4457000001e-1 * x(1)) * cos(x(5))) * cos(x(6)) + ((-0.7721000000e-1 * x(3) * cos(x(4)) + (0.1023032500e-1 + 0.7721000000e-1 * x(2)) * sin(x(4))) * sin(x(5)) + (0.5905525002e-2 + 0.4457000001e-1 * x(2)) * cos(x(4)) + 0.4457000001e-1 * x(3) * sin(x(4)) + (-0.1804706541e-1 + 0.7721000000e-1 * x(1)) * cos(x(5))) * sin(x(6))) * (((x(3) * cos(x(4)) * (-0.154420000000000002e0) + (x(2) * 0.154420000000000002e0 + 0.204606500000000004e-1) * sin(x(4))) * sin(x(5)) + (0.118110500000000000e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.360941307999999988e-1 + x(1) * 0.154420000000000002e0) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) - 0.118110500000000000e-1) * sin(x(4))) * sin(x(5)) + (x(2) * 0.154420000000000002e0 + 0.204606500000000004e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.154420000000000002e0 + (0.208355835999999989e-1 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + x(1) * (-0.467480000000000007e0) + x(2) * 0.265000000000000013e0 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2 + 0.801385065999999924e-1) ^ (-0.1e1 / 0.2e1) (((-0.4457000001e-1 * x(3) * cos(x(4)) + (-0.119746219040953224e-1 + 0.4457000001e-1 * x(2)) * sin(x(4))) * sin(x(5)) + (-0.7721000000e-1 * x(2) + 0.207440107070944561e-1) * cos(x(4)) - 0.7721000000e-1 * x(3) * sin(x(4)) + (-0.949341000324674535e-4 + 0.4457000001e-1 * x(1)) * cos(x(5))) * cos(x(6)) + ((-0.7721000000e-1 * x(3) * cos(x(4)) + (-0.207440107070944561e-1 + 0.7721000000e-1 * x(2)) * sin(x(4))) * sin(x(5)) + (-0.119746219040953224e-1 + 0.4457000001e-1 * x(2)) * cos(x(4)) + 0.4457000001e-1 * x(3) * sin(x(4)) + (-0.1644573001e-3 + 0.7721000000e-1 * x(1)) * cos(x(5))) * sin(x(6))) * (((x(3) * cos(x(4)) * (-0.154420000000000002e0) + (x(2) * 0.154420000000000002e0 - 0.414880214000000053e-1) * sin(x(4))) * sin(x(5)) + (-0.239492438000000023e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.328914600000000017e-3 + x(1) * 0.154420000000000002e0) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) + 0.239492438000000023e-1) * sin(x(4))) * sin(x(5)) + (x(2) * 0.154420000000000002e0 - 0.414880214000000053e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.154420000000000002e0 + (0.189868199999999979e-3 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + 0.801359748000000094e-1 + x(1) * (-0.425999999999999990e-2) + x(2) * (-0.537340000000000040e0) + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) (((-0.4457000001e-1 * x(3) * cos(x(4)) + (-0.119746219040953224e-1 + 0.4457000001e-1 * x(2)) * sin(x(4))) * sin(x(5)) + (-0.207440107070944561e-1 + 0.7721000000e-1 * x(2)) * cos(x(4)) + 0.7721000000e-1 * x(3) * sin(x(4)) + (0.949341000324674535e-4 + 0.4457000001e-1 * x(1)) * cos(x(5))) * cos(x(6)) + ((0.7721000000e-1 * x(3) * cos(x(4)) + (-0.7721000000e-1 * x(2) + 0.207440107070944561e-1) * sin(x(4))) * sin(x(5)) + (-0.119746219040953224e-1 + 0.4457000001e-1 * x(2)) * cos(x(4)) + 0.4457000001e-1 * x(3) * sin(x(4)) + (-0.1644573001e-3 - 0.7721000000e-1 * x(1)) * cos(x(5))) * sin(x(6))) * (((x(3) * cos(x(4)) * 0.154420000000000002e0 + (x(2) * (-0.154420000000000002e0) + 0.414880214000000053e-1) * sin(x(4))) * sin(x(5)) + (-0.239492438000000023e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.328914600000000017e-3 + x(1) * (-0.154420000000000002e0)) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) + 0.239492438000000023e-1) * sin(x(4))) * sin(x(5)) + (x(2) * (-0.154420000000000002e0) + 0.414880214000000053e-1) * cos(x(4)) + x(3) * sin(x(4)) * (-0.154420000000000002e0) + (-0.189868199999999979e-3 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + 0.801359748000000094e-1 + x(1) * 0.425999999999999990e-2 + x(2) * (-0.537340000000000040e0) + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2) ^ (-0.1e1 / 0.2e1) (((-0.4457000001e-1 * x(3) * cos(x(4)) + (0.5905525002e-2 + 0.4457000001e-1 * x(2)) * sin(x(4))) * sin(x(5)) + (0.1023032500e-1 + 0.7721000000e-1 * x(2)) * cos(x(4)) + 0.7721000000e-1 * x(3) * sin(x(4)) + (0.1041779181e-1 + 0.4457000001e-1 * x(1)) * cos(x(5))) * cos(x(6)) + ((0.7721000000e-1 * x(3) * cos(x(4)) + (-0.7721000000e-1 * x(2) - 0.1023032500e-1) * sin(x(4))) * sin(x(5)) + (0.5905525002e-2 + 0.4457000001e-1 * x(2)) * cos(x(4)) + 0.4457000001e-1 * x(3) * sin(x(4)) + (-0.1804706541e-1 - 0.7721000000e-1 * x(1)) * cos(x(5))) * sin(x(6))) * (((x(3) * cos(x(4)) * 0.154420000000000002e0 + (x(2) * (-0.154420000000000002e0) - 0.204606500000000004e-1) * sin(x(4))) * sin(x(5)) + (0.118110500000000000e-1 + x(2) * 0.891399999999999970e-1) * cos(x(4)) + x(3) * sin(x(4)) * 0.891399999999999970e-1 + (-0.360941307999999988e-1 + x(1) * (-0.154420000000000002e0)) * cos(x(5))) * cos(x(6)) + ((x(3) * cos(x(4)) * 0.891399999999999970e-1 + (x(2) * (-0.891399999999999970e-1) - 0.118110500000000000e-1) * sin(x(4))) * sin(x(5)) + (x(2) * (-0.154420000000000002e0) - 0.204606500000000004e-1) * cos(x(4)) + x(3) * sin(x(4)) * (-0.154420000000000002e0) + (-0.208355835999999989e-1 + x(1) * (-0.891399999999999970e-1)) * cos(x(5))) * sin(x(6)) + x(1) * 0.467480000000000007e0 + x(2) * 0.265000000000000013e0 + x(1) ^ 2 + x(2) ^ 2 + x(3) ^ 2 + 0.801385065999999924e-1) ^ (-0.1e1 / 0.2e1); 0 0 0 0 0 0; 0 0 0 0 0 0;];


end

