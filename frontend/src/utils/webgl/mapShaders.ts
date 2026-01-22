export const vertexShaderSource = `
  attribute vec2 a_position;
  attribute vec2 a_texCoord;
  
  varying vec2 v_texCoord;
  
  void main() {
    gl_Position = vec4(a_position, 0.0, 1.0);
    v_texCoord = a_texCoord;
  }
`;

export const fragmentShaderSource = `
  precision mediump float;
  
  uniform sampler2D u_image;
  uniform float u_occupiedThresh;
  uniform float u_freeThresh;
  uniform float u_showThreshold;
  uniform float u_showInflation;
  uniform float u_inflationRadius;
  uniform vec2 u_textureSize;
  
  varying vec2 v_texCoord;
  
  bool isOccupied(vec2 coord) {
    // Ensure coordinates are within bounds
    if (coord.x < 0.0 || coord.x > 1.0 || coord.y < 0.0 || coord.y > 1.0) {
      return false;
    }
    
    // Sample the texture at the given coordinate
    vec4 sampleColor = texture2D(u_image, coord);
    float gray = sampleColor.r; // Already 0-1 in WebGL texture
    
    // Check if it's black (occupied) - using a very low threshold for black pixels
    return gray < 0.1; // Threshold for black pixels (occupied areas)
  }
  
  float checkInflation(vec2 coord) {
    float radius = max(u_inflationRadius, 1.0); // Ensure minimum radius
    float inflationFound = 0.0;
    
    // Convert texture coordinate to pixel coordinate
    vec2 pixelCoord = coord * u_textureSize;
    
    // Check in a circular pattern around the current pixel
    for (float dy = -radius; dy <= radius; dy += 1.0) {
      for (float dx = -radius; dx <= radius; dx += 1.0) {
        float dist = length(vec2(dx, dy));
        if (dist <= radius) {
          // Convert back to texture coordinates
          vec2 samplePixel = pixelCoord + vec2(dx, dy);
          vec2 sampleCoord = samplePixel / u_textureSize;
          
          if (isOccupied(sampleCoord)) {
            inflationFound = 1.0;
            break;
          }
        }
      }
      if (inflationFound > 0.5) break;
    }
    
    return inflationFound;
  }
  
  void main() {
    vec4 color = texture2D(u_image, v_texCoord);
    float gray = color.r; // Values are already 0-1 in WebGL texture
    
    vec3 finalColor = vec3(gray);
    float alpha = 1.0;
    
    // Apply inflation visualization
    if (u_showInflation > 0.5) {
      // Check if current pixel is free space
      if (gray > 0.9) {
        // Check for inflation
        float inflated = checkInflation(v_texCoord);
        if (inflated > 0.5) {
          // Inflated areas - bright red with transparency
          finalColor = vec3(1.0, 0.0, 0.0);
          alpha = 0.7;
        } else {
          // Non-inflated free areas - keep original
          finalColor = vec3(gray);
          alpha = 0.0; // Make non-inflated areas transparent
        }
      } else if (gray < 0.1) {
        // Occupied areas - show as dark blue
        finalColor = vec3(0.0, 0.0, 0.8);
        alpha = 0.8;
      } else {
        // Unknown areas - show as yellow
        finalColor = vec3(1.0, 1.0, 0.0);
        alpha = 0.5;
      }
    }
    
    // Apply threshold visualization if enabled (and inflation is not)
    if (u_showThreshold > 0.5 && u_showInflation < 0.5) {
      float normalizedGray = gray; // Already 0-1
      if (normalizedGray < u_occupiedThresh) {
        finalColor = mix(vec3(gray), vec3(1.0, 0.0, 0.0), 0.5); // Red for occupied
      } else if (normalizedGray > u_freeThresh) {
        finalColor = mix(vec3(gray), vec3(0.0, 1.0, 0.0), 0.5); // Green for free
      } else {
        finalColor = mix(vec3(gray), vec3(1.0, 1.0, 0.0), 0.5); // Yellow for unknown
      }
    }
    
    gl_FragColor = vec4(finalColor, alpha);
  }
`;

export function createShader(gl: WebGLRenderingContext, type: number, source: string): WebGLShader | null {
  const shader = gl.createShader(type);
  if (!shader) return null;
  
  gl.shaderSource(shader, source);
  gl.compileShader(shader);
  
  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
    console.error('Shader compilation error:', gl.getShaderInfoLog(shader));
    gl.deleteShader(shader);
    return null;
  }
  
  return shader;
}

export function createProgram(gl: WebGLRenderingContext, vertexShader: WebGLShader, fragmentShader: WebGLShader): WebGLProgram | null {
  const program = gl.createProgram();
  if (!program) return null;
  
  gl.attachShader(program, vertexShader);
  gl.attachShader(program, fragmentShader);
  gl.linkProgram(program);
  
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
    console.error('Program linking error:', gl.getProgramInfoLog(program));
    gl.deleteProgram(program);
    return null;
  }
  
  return program;
}

export function setupWebGL(canvas: HTMLCanvasElement): {
  gl: WebGLRenderingContext;
  program: WebGLProgram;
  uniforms: { [key: string]: WebGLUniformLocation | null };
} | null {
  const gl = canvas.getContext('webgl', { 
    preserveDrawingBuffer: true,
    alpha: true,
    premultipliedAlpha: false
  }) || canvas.getContext('experimental-webgl', {
    preserveDrawingBuffer: true,
    alpha: true,
    premultipliedAlpha: false
  });
  if (!gl || !(gl instanceof WebGLRenderingContext)) {
    console.error('WebGL not supported');
    return null;
  }
  
  const vertexShader = createShader(gl, gl.VERTEX_SHADER, vertexShaderSource);
  const fragmentShader = createShader(gl, gl.FRAGMENT_SHADER, fragmentShaderSource);
  
  if (!vertexShader || !fragmentShader) return null;
  
  const program = createProgram(gl, vertexShader, fragmentShader);
  if (!program) return null;
  
  // Set up vertices for a full-screen quad
  // Note: Flip texture coordinates vertically to match canvas coordinate system
  const vertices = new Float32Array([
    -1.0, -1.0, 0.0, 1.0,  // bottom-left vertex -> top-left texcoord
     1.0, -1.0, 1.0, 1.0,  // bottom-right vertex -> top-right texcoord
    -1.0,  1.0, 0.0, 0.0,  // top-left vertex -> bottom-left texcoord
     1.0,  1.0, 1.0, 0.0,  // top-right vertex -> bottom-right texcoord
  ]);
  
  const buffer = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
  gl.bufferData(gl.ARRAY_BUFFER, vertices, gl.STATIC_DRAW);
  
  const positionLocation = gl.getAttribLocation(program, 'a_position');
  const texCoordLocation = gl.getAttribLocation(program, 'a_texCoord');
  
  gl.useProgram(program);
  
  // Position attribute
  gl.enableVertexAttribArray(positionLocation);
  gl.vertexAttribPointer(positionLocation, 2, gl.FLOAT, false, 16, 0);
  
  // Texture coordinate attribute
  gl.enableVertexAttribArray(texCoordLocation);
  gl.vertexAttribPointer(texCoordLocation, 2, gl.FLOAT, false, 16, 8);
  
  // Get uniform locations
  const uniforms = {
    u_image: gl.getUniformLocation(program, 'u_image'),
    u_occupiedThresh: gl.getUniformLocation(program, 'u_occupiedThresh'),
    u_freeThresh: gl.getUniformLocation(program, 'u_freeThresh'),
    u_showThreshold: gl.getUniformLocation(program, 'u_showThreshold'),
    u_showInflation: gl.getUniformLocation(program, 'u_showInflation'),
    u_inflationRadius: gl.getUniformLocation(program, 'u_inflationRadius'),
    u_textureSize: gl.getUniformLocation(program, 'u_textureSize'),
  };
  
  return { gl, program, uniforms };
} 