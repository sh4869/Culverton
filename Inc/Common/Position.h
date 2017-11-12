#ifndef POSITION_H_
#define POSITION_H_

/**
 * Position
 */
struct Position {
    float x;
    float y;
    float theta;
    /**
     * @brief コンストラクタ
     *
     * @param 0.0f
     * @param 0.0f
     * @param 0.0f
     */
    Position(float _x = 0.0f, float _y = 0.0f, float _theta = 0.0f) : x(_x), y(_y), theta(_theta){};

    /**
     * @brief + operator
     *
     * @param other
     * @return Position
     */
    Position operator+(Position other) const {
        return Position(this->x + other.x, this->y + other.y, this->theta + other.theta);
    }

    /**
     * @brief - operator
     *
     * @param other
     * @return Position
     */
    Position operator-(Position other) const {
        return Position(this->x - other.x, this->y - other.y, this->theta - other.theta);
    }
};

#endif