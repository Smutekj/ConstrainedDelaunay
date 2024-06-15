#version 120

uniform sampler2D image1;
uniform sampler2D image2;


uniform float offset[5] = float[]( 0.0, 1.0, 2.0, 3.0, 4.0 );
uniform float weight[5] = float[]( 0.2270270270, 0.1945945946, 0.1216216216, 0.0540540541, 0.0162162162 );

void main(void)
{
	vec2 texCoords = gl_TexCoord[0].xy;
	vec4 vertexColor = gl_Color;

	gl_FragColor = 5.*texture2D( image1,  vec2(texCoords.x, texCoords.y)) + texture2D( image2, vec2(texCoords.x, texCoords.y));

}
