#pragma once
#include <cstddef>

#define WHITE 0
#define GRAY 1
#define BLACK 2
#define G1 1
#define G2 2
class Vertex{
public:
	Vertex() = default;
	Vertex(const size_t name);
	void setPi(int pi);
	void setTimeIn(const size_t time_in);
	void setTimeOut(const size_t time_out);
	void setName(const size_t name);
	void setWhite();
	void setGrey();
	void setBlack();
	void changeStatus(int status);

	int getStatus() const;
	size_t getName() const;
	int getPi() const;
	size_t getTimeIn() const;
	size_t getTimeOut() const;
	size_t getColor() const;

	bool operator< (const Vertex& other) const{
		return name_ < other.name_;
	}

private:
	size_t name_ = 0;
	size_t time_in_ = 0;
	size_t time_out_ = 0;
	int pi_ = -1;
	size_t color_ = WHITE;
	size_t status_ = G1;
};