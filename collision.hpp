#ifndef _COLLISION_HPP
#define _COLLISION_HPP

#include "type_traits.hpp"

#include "vector.hpp"
#include "shapes.hpp"
#include <limits>

/*
	Line Segment vs Circle 

	Swept Collision Detection



	Goal: 

	Calculate the points of collision 
	for a static line segment and 
	accelerating circle 
	over a time step.



	Obstacles:

	There are two cases.  
	A circle may collide
	with one of the end points or
	a point in between.

	These cases require different logic when
	locating intersection points on the circle and
	line segment.

	Distinguishing these cases is tricky.  
	How can we tell if the circle is going to
	collide with one of the end points or 
	a point in between?



	Solution:

	Part of the following code does this by checking 
	if the dot product of the circle velocity onto the segment is within 
	the dot products of the segment onto itself.
	
	

	TODO: Store the point of collision of the circle for all static and semi static lines.

	TODO: Test performance of this against other algorithms.  Find a legitimate way of doing this.

	TODO: Exit early if velocity vector points away such that no collision would be possible.

	TODO: Ability to check and resolve collision between dynamic segments vs circle and circle vs circle.
*/

namespace uti {

	// Returns value to be passed to get the point or end point on a line or segment closest to a circle
	Scalar getRoot( 			cr< Point >, 	cr< Segment > );

	// Returns end point on a segment closest to a circle
	Point  getClosestEndPoint( 	Scalar, 		cr< Segment > );

	// Returns point on a line closest to a circle
	Point  getClosestPoint( 	Scalar, 		cr< Segment > );

	// Returns point on a circle closest to a point 
	Point  getClosestPoint( 	cr< Circle >, 	cr< Point > );

	// Segment vs Segment Point of Intersection
	bool   getIntersection(		cr< Segment >, 	cr< Segment >, Point & );

	// Circle vs Segment Point of Intersection
	bool   getIntersection( 	cr< Circle >, 	cr< Segment >, Point & );

	enum CollisionType {
		None,
		Between,
		EndPoint
	};

	CollisionType isCollision(
		cr< Circle >  	circle,
		cr< Vector >	velocity,
		cr< Segment >	segment,
		Point &    		circleContactPoint,
		Point &    		segmentContactPoint
	) {
		const Vector line{ segment.end - segment.begin };
		
		const Scalar root{ getRoot( circle.center, segment ) };
		
		const Point closestPointOnLine{ getClosestPoint( root, segment ) };
		
		circleContactPoint = getClosestPoint( circle, closestPointOnLine );
		bool is_intersection{ getIntersection( { circleContactPoint, circleContactPoint + velocity }, segment, segmentContactPoint ) };

		const Scalar lower{ segment.begin * line };
		const Scalar upper{ segment.end * line };
		const Scalar proj{ segmentContactPoint * line };

		if( is_intersection && ( proj - lower ) * ( proj - upper ) <= 0 ) { 
			return CollisionType::Between;
		}

		segmentContactPoint = getClosestEndPoint( root, segment );
		is_intersection = getIntersection( circle, { segmentContactPoint, segmentContactPoint - velocity }, circleContactPoint );
		
		if( is_intersection ) { 
			return CollisionType::EndPoint; 
		}

		return CollisionType::None;
	}

	inline Scalar getRoot( cr< Point > point, cr< Segment > segment ) {
		const Vector line{ segment.begin - segment.end };
		return line * ( point - segment.end ) / ( line * line );
	}
	inline Point getClosestEndPoint( Scalar root, cr< Segment > segment ) {  
		if( root < 0 ) return segment.end;
		if( root > 0 ) return segment.begin;
	}
	inline Point getClosestPoint( Scalar root, cr< Segment > segment ) {  
		return root * segment.begin + ( 1 - root ) * segment.end;
	}
	inline Point getClosestPoint( cr< Circle > circle, cr< Point > point ) {
		const Vector u{ hat( point - circle.center ) };
		return u * circle.radius + circle.center;
	}
	inline bool getIntersection( cr< Segment > lhs, cr< Segment > rhs, Point & poi ) {
		const Vector d{ lhs.end - lhs.begin };
		const double m{ d.x != 0 ? d.y / d.x : std::numeric_limits< double >::quiet_NaN() };
		const double b{ lhs.begin.y - m * lhs.begin.x };
		const Vector f{ rhs.end - rhs.begin };
		const double t{ f.x != 0 ? f.y / f.x : std::numeric_limits< double >::quiet_NaN() };
		const double c{ rhs.begin.y - t * rhs.begin.x };
		const double x{ ( b - c ) / ( t - m ) };
		const double y{ m * x + b };
		if( isnan( m ) && ! isnan( t ) ) {
			poi = { lhs.end.x, t * lhs.end.x + c };
		} else if( isnan( t ) && ! isnan( m ) ) {
			poi = { rhs.end.x, m * rhs.end.x + b };
		} else {
			const double x{ ( b - c ) / ( t - m ) };
			const double y{ m * x + b };
			poi = { x, y };
		}
		const bool v{ ( poi.x - lhs.begin.x ) * ( poi.x - lhs.end.x ) * ( poi.y - lhs.begin.y ) * ( poi.y - lhs.end.y ) >= 0 };
		const bool g{ ( poi.x - rhs.begin.x ) * ( poi.x - rhs.end.x ) * ( poi.y - rhs.begin.y ) * ( poi.y - rhs.end.y ) >= 0 };
		return v && g;
	}
	inline bool getIntersection( cr< Circle > circle, cr< Segment > segment, Point& poi ) {
		const Vector d{ segment.end - segment.begin };
		const Vector f{ segment.begin - circle.center };
		const double a{ d * d };
		const double b{ 2 * f * d };
		const double c{ f * f - circle.radius * circle.radius };
		const double u{ b * b - 4 * a * c };
		if( u < 0 ) return false;
		const double root{ ( -b - sqrt( u ) ) / ( 2 * a ) };
		if( 0 > root && root > 1 ) return false;
		poi = segment.begin + root * d;
		return true;
	}
};

#endif
