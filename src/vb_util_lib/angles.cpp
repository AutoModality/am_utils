#include <vb_util_lib/angles.h>
#include <vb_util_lib/am_geometry.h>


namespace am
{

int Angles::compare(double ang1, double ang2)
{
	double a1 = AM::wrap_pi(ang1);
	double a2 = AM::wrap_pi(ang2);
	double opposite = AM::wrap_pi(a1 + M_PI);

	if(a1 == a2)
	{
		return 0.0;
	}
	else if(a1 > 0)
	{
		if(a2 > a1)
		{
			return -1;
		}
		else if(a2 > opposite)
		{
			return 1;
		}
		else
		{
			return -1;
		}
	}
	else
	{
		if(a2 < a1)
		{
			return 1;
		}
		else if(a2 < opposite)
		{
			return -1;
		}
		else
		{
			return 1;
		}
	}
}

double Angles::getDiffSigned(double ang1, double ang2)
{
	double delta = getDiff(ang1, ang2);

	if(compare(ang1, ang2) >= 0.0)
	{
		return -delta;
	}
	else
	{
		return delta;
	}
}

double Angles::getDiff(double ang1, double ang2)
{
        double a1 = AM::wrap_pi(ang1);
        double a2 = AM::wrap_pi(ang2);
        double delta = M_PI - std::abs( std::abs(a1 - a2) - M_PI);
        return delta;
}

double Angles::getOrderedDiff(double ang1, double ang2)
{
        double a1 = AM::wrap_2pi(ang1);
        double a2 = AM::wrap_2pi(ang2);
        if(a1 < a2)
        {
          a1 += 2 * M_PI;
        }
        return a1 - a2;
}

bool Angles::isBetween(double target, double left_angle, double right_angle, double & clamped)
{
	double la = AM::wrap_pi(left_angle);
	double ra = AM::wrap_pi(right_angle);

	double left_delta = getDiff(target, la);
	double right_delta = getDiff(target, ra);

	double closest_angle = left_delta < right_delta ? la : ra;

	double ln = la < 0.0 ? la + 2 * M_PI : la;
	double rn = ra < 0.0 ? ra + 2 * M_PI : ra;
	double tn = target < 0.0 ? target + 2 * M_PI : target;
	if(rn > ln)
	{
		if(tn > rn || tn < ln)
		{
			clamped = target;
			return true;
		}
		else
		{
			clamped = closest_angle;
			return false;
		}
	}
	else
	{
		if(tn > rn && tn < ln)
		{
			clamped = target;
			return true;
		}
		else
		{
			clamped = closest_angle;
			return false;
		}
	}
}

}
