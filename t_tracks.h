#pragma once
class t_tracks
{
public:
	int id;
	int bbox;
	int kalmaFilter;
	int age;
	int totalVisibleCount;
	int consecutiveInvisibleCount;

	t_tracks() {};
	~t_tracks() {};
};

