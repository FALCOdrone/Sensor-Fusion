function quatToEuler = fromQuatToEuler(q)
    sequence = 'XYZ';
    quatToEuler = quat2eul(q, sequence);
end