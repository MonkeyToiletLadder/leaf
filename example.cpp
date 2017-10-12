#include "collision.hpp"

#include <iostream>

#include <chrono>

std::ostream& operator<<( std::ostream& os, uti::cr< uti::Point > p ) { return os << '<' << p.x << ',' << p.y << '>'; }

/*
	I use a Segment class to make it less confusing at to
	what is being passed, but this is not required.  
	The functions originally accepted end points which 
	results in faster execution times ( 4 - 5 microseconds ).
*/

int main(){
	using namespace uti;

	// Curently the velocity remains the same after reflection.
	// There is also no acceleration / deacceleration implemented yet.

	Circle circle{ { 0,0 }, 1 };
	Vector velocity{ 1.29289,1.29289 };
	double elapsed{ 2 };

	Segment segment{ {2,2}, {4,2} };

	const Vector step{ elapsed * velocity };
	const double speed{ mag( step ) };
	const Vector line{ segment.end - segment.begin };
	
	// This is used for reflection, these segments normals can be precalculated if they are static ( non moving ).

	const Vector normal{ nrm( hat( line ) ) };


	std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
	
	// Collision Points

	Point circlecol,segmentcol;

	// Get the point of collision for the segment and circle over a time step.

	CollisionType collision_type{ isCollision( circle, step, segment, circlecol, segmentcol ) };

	// Determine behavior based on type of collision.

	switch( collision_type ){

	// Collision Resolution by reflection or wall bounce

	case CollisionType::Between:
	{ // Reflect off the segments normal.
		const Vector velocity{ segmentcol - circlecol };
		const Vector scale{ step * ( ( speed - mag( velocity ) ) / speed ) };
		const Vector reflection{ scale - 2 * ( scale * normal ) * normal };
		circle.center = circle.center + velocity + reflection; 
		break;
	}

	case CollisionType::EndPoint:
	{ // Reflect off the tangents, runs through the intersection, normal.
		const Vector velocity{ segmentcol - circlecol };
		const Vector tangent{ circlecol - circle.center };
		const Vector normal{ hat( tangent ) };
		const Vector scale{ step * ( ( speed - mag( velocity ) ) / speed ) };
		const Vector reflection{ scale - 2 * ( scale * normal ) * normal };
		circle.center = circle.center + velocity + reflection; 
		break;
	}

	case CollisionType::None:
	{ // No collision detected move by velocity.
		circle.center = circle.center + step; 
		break;
	}

	default:
		;
	};

	std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

	auto duration{ std::chrono::duration_cast< std::chrono::microseconds >( t2 - t1 ).count() };
	std::cout << "Execution Time\t" << duration << std::endl;
	std::cout << "New Position\t" << circle.center << std::endl;
}