#pragma once
#include <string>

class Shader
{
public:
	Shader(const std::string& vertexPath, const std::string& fragmentPath);
	~Shader();
	void Use();
	inline const uint32_t GetID() { return m_ID; }
private:
	unsigned int m_ID;
};

