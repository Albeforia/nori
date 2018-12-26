#include <nori/texture.h>

NORI_NAMESPACE_BEGIN

class Checkerboard : public Texture2D<Color3f> {

public:
	Checkerboard(const PropertyList& props) :
	    Texture2D(props) {
		m_color0 = props.getColor("value", Color3f(0.4f));
		m_color1 = props.getColor("value", Color3f(0.2f));
	}

	Color3f eval(const Point2f& uv) const override {
		Point2f uv2 = Point2f(uv.x() * m_uvScale.x(), uv.y() * m_uvScale.y()) + m_uvOffset;

		// map to -1, 1
		int x = 2 * mod((int)(uv2.x() * 2), 2) - 1,
		    y = 2 * mod((int)(uv2.y() * 2), 2) - 1;

		if (x * y == 1)
			return m_color0;
		else
			return m_color1;
	}

	std::string toString() const {
		return tfm::format(
		  "Checkerboard[\n"
		  "  offset = %s,\n"
		  "  scale = %s,\n"
		  "  color0 = %s,\n"
		  "  color1 = %s\n"
		  "]",
		  indent(m_uvOffset.toString()),
		  indent(m_uvScale.toString()),
		  indent(m_color0.toString()),
		  indent(m_color1.toString()));
	}

private:
	Color3f m_color0;
	Color3f m_color1;
};

NORI_REGISTER_CLASS(Checkerboard, "checkerboard");
NORI_NAMESPACE_END
