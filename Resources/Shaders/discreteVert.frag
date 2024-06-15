#version 120

uniform sampler2D image;


uniform float offset[5] = float[]( 0.0, 1.0, 2.0, 3.0, 4.0 );
uniform float weight[5] = float[]( 0.2270270270, 0.1945945946, 0.1216216216, 0.0540540541, 0.0162162162 );

void main(void)
{
	vec2 texCoords = gl_TexCoord[0].xy;
	vec4 vertexColor = gl_Color;

	gl_FragColor = texture2D( image, vec2(texCoords)) * weight[0];
	for (int i=1; i<5; i++)
	{
		gl_FragColor += texture2D( image, ( vec2(texCoords)+vec2(0.0, offset[i]/600) ) ) * weight[i];
		gl_FragColor += texture2D( image, ( vec2(texCoords)-vec2(0.0, offset[i]/600) ) ) * weight[i];
	}
	//gl_FragColor.a = (gl_FragColor.r + gl_FragColor.g + gl_FragColor.b)/3.f;

}
