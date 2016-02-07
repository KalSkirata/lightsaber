
#ifdef __cplusplus
extern "C" {
#endif
#include "glimage.h"
#ifdef __cplusplus
}
#endif

#include <SDL/SDL_image.h>
#include <assert.h>

static void FlipVertically(SDL_Surface *);

static 
void FlipVertically(SDL_Surface *tex){
  unsigned int size = tex->pitch;
  char *data = NULL;
  char *a = NULL, *b = NULL;

  data =  (char*) malloc(size);
  assert(data);

  a = (char*)tex->pixels;
  b = (char*)tex->pixels + size*(tex->h-1);
  
  while(a < b){
    memcpy(data, a, size); 
    memcpy(a, b ,size);  
    memcpy(b, data, size); 
    a += size; b -= size;
  }
  free(data);
}

/**
	\func associe une image à une texture 2D
	\param fname chemin vers l'image
	\param tex id de la texture
**/
extern
int glimageLoadAndBind (const char * fname, GLuint *tex) {

  SDL_Surface * texSurface = NULL;  
  SDL_RWops *rwop = NULL;
  char ShouldBeFlipped = 0;
  
  int flags= IMG_INIT_JPG | IMG_INIT_PNG;
  int initted= IMG_Init(flags);
  int sign = (SDL_BYTEORDER == SDL_BIG_ENDIAN) ? 1 : -1;

  printf("%s\n", fname);
  if((initted & flags) != flags) {
    printf("IMG_Init: Failed to init required jpg and png support!\n");
    printf("IMG_Init: %s\n", IMG_GetError());
    return 0;
  }

  if((texSurface = IMG_Load(fname)) == NULL ) {
    fprintf(stderr, "Impossible d'ouvrir le fichier : %s\n", IMG_GetError());
    return 0;
  }
  rwop=SDL_RWFromFile(fname, "rb");
  
  //Si l'image est en JPEG ou PNG, il faut inverser
  if (IMG_isJPG(rwop) || IMG_isBMP(rwop) || IMG_isPNG(rwop)) ShouldBeFlipped = 1;
    
  if (ShouldBeFlipped) //invert_surface_vertical(texSurface); 
    FlipVertically(texSurface); 

  IMG_Quit();
    
  glGenTextures(1, tex);
  
  glBindTexture(GL_TEXTURE_2D, *tex); 

  if (texSurface->format->BytesPerPixel == 3){
	if (sign * texSurface->format->Rshift > sign * texSurface->format->Bshift)
	  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texSurface->w, texSurface->h, 0, GL_RGB, GL_UNSIGNED_BYTE, texSurface->pixels);
	else 
	  glTexImage2D(GL_TEXTURE_2D, 0,  GL_RGB, texSurface->w, texSurface->h, 0, GL_BGR, GL_UNSIGNED_BYTE, texSurface->pixels);
  }

  if (texSurface->format->BytesPerPixel == 4){
	if (sign * texSurface->format->Rshift > sign * texSurface->format->Bshift)
	  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texSurface->w, texSurface->h, 0, GL_RGBA, GL_UNSIGNED_BYTE, texSurface->pixels);
	else 
	  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texSurface->w, texSurface->h, 0, GL_BGRA, GL_UNSIGNED_BYTE, texSurface->pixels);
  }


  SDL_FreeSurface(texSurface);
  return 1;
}

