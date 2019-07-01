
#include <libxpat/nav_data/NavPoint.hpp>
#include <glm/geometric.hpp>
#include <cmath>

using namespace xpat::nav;
using namespace xpat::phys;
using namespace units;


xpat::nav::NavPoint::NavPoint(const double& latitude, const double& longitude, const double& elev) noexcept : NavPoint(degrees(latitude), degrees(longitude), feet(elev))
{
}

xpat::nav::NavPoint::NavPoint(const phys::degrees& latitude, const phys::degrees& longitude, const phys::feet& elevation) noexcept : latitude(latitude), longitude(longitude), elevation_amsl(elevation) {}

xpat::nav::NavPoint::NavPoint(const NavPoint& nav_point, const phys::feet& elevation) noexcept : NavPoint(nav_point.latitude, nav_point.longitude, elevation) {}

glm::dvec3 xpat::nav::NavPoint::as_euclidean_coord() const {
    glm::dvec2 base{ this->latitude.convert<units::angle::radians>().to<double>(),
        this->longitude.convert<units::angle::radians>().to<double>() };

    auto out = glm::euclidean(base);
    out *= earth_mean_radius.convert<units::length::meters>().to<double>();
    return out;
}

nautical_miles xpat::nav::NavPoint::euclidean_distance(const NavPoint& that) const
{
    const auto coord_a = this->as_euclidean_coord();
    const auto coord_b = that.as_euclidean_coord();

    return meters(glm::distance(coord_a, coord_b));
}

feet xpat::nav::NavPoint::vertical_distance(const NavPoint& that) const noexcept
{
    return math::fabs(this->elevation_amsl - that.elevation_amsl);
}

nautical_miles xpat::nav::NavPoint::slant_range(const NavPoint& that) const noexcept
{
    return math::sqrt(math::cpow<2>(this->haversine_distance(that)) + math::cpow<2>(this->vertical_distance(that)));
}

NavPoint xpat::nav::NavPoint::translate(const radians& bearing, const meters& distance, const feet& altitude_change) const noexcept
{
    return NavPoint(this->lateral_translate(bearing, distance), this->elevation_amsl + altitude_change);
}

NavPoint xpat::nav::NavPoint::normalize() const noexcept
{
    return polar_math::normalize_nav_point(*this);
}

NavPoint xpat::nav::NavPoint::lateral_translate(const radians& bearing, const meters& distance) const noexcept
{
    auto R = WGS84::radius_at_latitude(latitude);
    radians dist_ratio{(distance / R).to<double>()};
    auto lat_sin = math::sin(latitude);
    auto lat_cos = math::cos(latitude);
    auto distrat_cos = math::cos(dist_ratio);
    auto distrat_sin = math::sin(dist_ratio);
    degrees new_lat = math::asin(
        lat_sin * distrat_cos +
        lat_cos * distrat_sin * math::cos(bearing)
    );
    degrees new_lon = longitude + math::atan2(
        math::sin(bearing) * distrat_sin * lat_cos,
        distrat_cos - lat_sin * math::sin(new_lat)
    );

    return polar_math::normalize_nav_point(NavPoint(new_lat, new_lon));
}

nautical_miles NavPoint::haversine_distance(const NavPoint& other) const noexcept
{
    // Use the Haversine formula

    degrees delta_lat = other.latitude - this->latitude;
    degrees delta_lon = other.longitude - this->longitude;

    dimensionless::scalar_t a = math::cpow<2>(math::sin(delta_lat / 2.0)) +
        (math::cpow<2>(math::sin(delta_lon / 2.0)) *
            math::cos(other.latitude) * math::cos(this->latitude));

    angle::radian_t c = 2.0 * math::atan2(math::sqrt(a), math::sqrt(1.0 - a));

    meters dist = R_e * c.to<double>();

    return dist;
}

// Based on https://www.movable-type.co.uk/scripts/latlong-vincenty.html
std::optional<nautical_miles> xpat::nav::NavPoint::vincenty_distance(const NavPoint& that, unsigned iteration_limit, const radians& precision) const noexcept
{
    radians delta_lon(that.longitude - this->longitude);
    auto tanU1 = (1.0 - WGS84::flattening) * math::tan(this->latitude);
    auto cosU1 = 1.0 / math::sqrt(1.0 + math::cpow<2>(tanU1));
    auto sinU1 = tanU1 * cosU1;
    auto tanU2 = (1.0 - WGS84::flattening) * math::tan(that.latitude);
    auto cosU2 = 1.0 / math::sqrt(1.0 + math::cpow<2>(tanU2));
    auto sinU2 = tanU2 * cosU2;

    radians lambda, prev_lambda;
    dimensionless::scalar_t cos_sqalpha, cos_2sigmaM, sin_sigma, cos_sigma, sigma;
    lambda = delta_lon;
    do {

        auto sin_lam = math::sin(lambda);
        auto cos_lam = math::cos(lambda);
        auto sin_sqsigma = math::cpow<2>(cosU2 * sin_lam) + math::cpow<2>(cosU1 * sinU2 - sinU1 * cosU2 * cos_lam);
        sin_sigma = math::sqrt(sin_sqsigma);
        if (sin_sigma.to<double>() == 0.0) {
            return miles(0);
        }

        cos_sigma = sinU1 * sinU2 + cosU1 * cosU2 * cos_lam;
        sigma = dimensionless::scalar_t(math::atan2(sin_sigma, cos_sigma).to<double>());
        auto sin_alpha = cosU1 * cosU2 * sin_lam / sin_sigma;
        cos_sqalpha = 1 - math::cpow<2>(sin_sigma);
        cos_2sigmaM = cos_sigma - 2.0 * sinU1 * sinU2 / cos_sqalpha;

        if (std::isnan(cos_2sigmaM.to<double>())) {
            cos_2sigmaM = decltype(cos_2sigmaM)(0.0);
        }

        auto C = WGS84::flattening / 16.0 * cos_sqalpha * (4.0 + WGS84::flattening * (4.0 - 3.0 * cos_sqalpha));
        prev_lambda = lambda;
        lambda = delta_lon + radians(((1.0 - C) * WGS84::flattening * sin_alpha * (sigma + C * sin_sigma * (cos_2sigmaM + C * cos_sigma * (-1.0 + 2.0 * math::cpow<2>(cos_2sigmaM))))).to<double>());

        iteration_limit--;
    } while (math::fabs(lambda - prev_lambda) > precision && iteration_limit > 0);
    // Rather than fail if we can't converge, we'll default to haversine distance. Not ideal, but less expensive than throwing.
    if (iteration_limit == 0) {
        return std::optional<nautical_miles>();
    }

    auto u_sq = cos_sqalpha * (math::cpow<2>(WGS84::equatorial_radius) - math::cpow<2>(WGS84::polar_radius)) / (math::cpow<2>(WGS84::polar_radius));
    auto A = 1.0 + u_sq / 16384.0 * (4096.0 + u_sq * (-768.0 + u_sq * (320 - 175 * u_sq)));
    auto B = u_sq / 1024.0 * (256.0 + u_sq * (-128.0 + u_sq * (74 - 47 * u_sq)));
    auto delta_sigma = B * sin_sigma * (cos_2sigmaM + B / 4.0 * (cos_sigma * (-1.0 + 2.0 * math::cpow<2>(cos_2sigmaM)) - B / 6.0 * cos_2sigmaM * (-3.0 + 4.0 * math::cpow<2>(sin_sigma)) * (-3.0 + 4.0 * math::cpow<2>(cos_2sigmaM))));

    const nautical_miles s = WGS84::polar_radius * A * (sigma - delta_sigma);

    return std::make_optional(s);
    
}

degrees xpat::nav::NavPoint::bearing_to(const NavPoint& that) const noexcept
{
    auto delta_lon = that.longitude - this->longitude;
    auto y = math::cos(that.latitude) * math::sin(delta_lon);
    auto x = math::cos(this->latitude) * math::sin(that.latitude) - math::sin(this->latitude) * math::cos(that.latitude) * math::cos(delta_lon);

    double res = degrees(math::atan2(y, x)).to<double>();
    res += 360.0;
    while (res > 359.99999) {
        res -= 360.0;
    }

    return degrees(res);
}

degrees xpat::nav::polar_math::normalize_longitude(const phys::degrees& longitude) noexcept {
    auto new_lon = normalize_heading(longitude);
    if (new_lon > semi_circle) {
        new_lon -= full_circle;
    }
    else if (new_lon <= -semi_circle) {
        new_lon += full_circle;
    }
    return new_lon;
}

NavPoint xpat::nav::polar_math::normalize_nav_point(const NavPoint& nav) noexcept {
    NavPoint out;
    out.longitude = normalize_longitude(nav.longitude);
    auto new_lat = normalize_longitude(nav.latitude);
    bool flip = false;
    if (new_lat > right_angle) {
        new_lat = semi_circle - new_lat;
        flip = true;
    }
    else if (new_lat <= -right_angle) {
        new_lat = -semi_circle - new_lat;
        flip = true;
    }

    if (flip) {
        out.longitude *= -1.0;
    }

    out.latitude = new_lat;

    return out;
}

meters xpat::nav::WGS84::radius_at_latitude(const phys::radians& latitude) noexcept {
    auto cos_lat = units::math::cos(latitude);
    auto sin_lat = units::math::cos(latitude);

    return units::math::sqrt((units::math::cpow<2>(units::math::cpow<2>(WGS84::equatorial_radius) * cos_lat) + units::math::cpow<2>(units::math::cpow<2>(WGS84::polar_radius) * sin_lat)) /
        (units::math::cpow<2>(WGS84::equatorial_radius * cos_lat) + units::math::cpow<2>(WGS84::polar_radius * sin_lat)));
}
