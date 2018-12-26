#include <nori/texture.h>

NORI_NAMESPACE_BEGIN

class ConstantTexture : public Texture2D<Color3f> {

public:
	ConstantTexture(const PropertyList& props) :
	    Texture2D(props) {
		m_value = props.getColor("value", Color3f(0.5f));
	}

	Color3f eval(const Point2f& uv) const override {
		return m_value;
	}

	std::string toString() const {
		return tfm::format(
		  "ConstantTexture[\n"
		  "  value = %s\n"
		  "]",
		  indent(m_value.toString()));
	}

private:
	Color3f m_value;
};

NORI_REGISTER_CLASS(ConstantTexture, "constexture");
NORI_NAMESPACE_END
