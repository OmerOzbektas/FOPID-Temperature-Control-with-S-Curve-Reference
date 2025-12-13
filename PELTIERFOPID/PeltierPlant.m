function G = PeltierPlant(p, padeOrder, variation)

    if nargin < 2 || isempty(padeOrder)
        padeOrder = 2;
    end
    if nargin < 3
        variation = struct();
    end

    Kth = p.Kth;
    tau = p.tau;
    Td  = p.Td;

    if isfield(variation, 'Kth')
        Kth = Kth * (1 + variation.Kth);
    end
    if isfield(variation, 'tau')
        tau = tau * (1 + variation.tau);
    end
    if isfield(variation, 'Td')
        Td = Td * (1 + variation.Td);
    end

    num = Kth;
    den = [tau 1];
    G_nom = tf(num, den);

    if Td > 0
        [numD, denD] = pade(Td, padeOrder);
        D = tf(numD, denD);
        G = series(D, G_nom);
    else
        G = G_nom;
    end
end
