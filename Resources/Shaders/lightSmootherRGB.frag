#version 130

uniform sampler2D image;
uniform bool vertical = true;


uniform float offset[5] = float[]( 0.0, 1.0, 2.0, 3.0, 4.0 );
uniform float weight[5] = float[]( 0.2270270270, 0.1945945946, 0.1216216216, 0.0540540541, 0.0162162162 );

void main(void)
{
	vec2 texCoords = gl_TexCoord[0].xy;
	vec4 vertexColor = gl_Color;

	vec2 size = textureSize(image, 0);

	vec3 result  = texture2D( image, texCoords ).rgb * weight[0];
	if(vertical)
	{
		for (int i=1; i<5; i++) {
			result += texture2D( image, ( vec2(texCoords)+vec2(0.0, offset[i]/size.y) ) ).rgb * weight[i];
			result += texture2D( image, ( vec2(texCoords)-vec2(0.0, offset[i]/size.y) ) ).rgb * weight[i];
		}
	}else
	{
		for (int i=1; i<5; i++) {
			result += texture2D( image, ( texCoords+vec2(offset[i]/size.x, 0.0) ) ).rgb * weight[i];
			result += texture2D( image, ( texCoords-vec2(offset[i]/size.x, 0.0) ) ).rgb * weight[i];
		}
	}
	gl_FragColor = vec4(result, 1.0);
}
