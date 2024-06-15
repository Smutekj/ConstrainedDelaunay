#version 130

uniform sampler2D image;

void main()
{

	ivec2 size = textureSize(image, 0);
	vec2 texCoords = gl_TexCoord[0].xy;
	vec4 sourceFragment00 = texture2D(image, texCoords );
	vec4 sourceFragment10 = texture2D(image, texCoords + vec2(1./size.x, 0.));
	vec4 sourceFragment01 = texture2D(image, texCoords + vec2(0., 1./size.y) ) ;
	vec4 sourceFragment11 = texture2D(image, texCoords + vec2(1./size.x, 1./size.y));
	gl_FragColor = (sourceFragment00 + sourceFragment10 + sourceFragment01 + sourceFragment11)/4.;
}