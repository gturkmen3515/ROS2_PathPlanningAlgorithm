//
// Created by atakan on 19.06.2023.
//

#ifndef CPP_PUBSUB_ISCELLONLINE_H
#define CPP_PUBSUB_ISCELLONLINE_H
bool isCellOnLine(int x1, int y1, int x2, int y2)
{
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true)
    {
        if (x1 == x2 && y1 == y2)
        {
            return true;  // The cell lies on the line segment
        }

        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
}


#endif //CPP_PUBSUB_ISCELLONLINE_H
