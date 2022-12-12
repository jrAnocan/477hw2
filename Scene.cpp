#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <cmath>

#include "Scene.h"
#include "Camera.h"
#include "Color.h"
#include "Mesh.h"
#include "Rotation.h"
#include "Scaling.h"
#include "Translation.h"
#include "Triangle.h"
#include "Vec3.h"
#include "tinyxml2.h"
#include "Helpers.h"

using namespace tinyxml2;
using namespace std;


int Scene::getIndexOfMin( Vec3& u)
{
	int index_of_min = 0,min = 999999999;
	double arr[3] = {u.x,u.y,u.z};
	
	for(int i=0;i<3;i++)
	{
		if(arr[i]<min)
		{
			min = arr[i];
			index_of_min = i;
		}
	}
	return index_of_min;
}

Matrix4 Scene::getTranslationMatrix( Translation& t)
{
	double translation_matrix[4][4] = 
	{
		{1.0,0,0,t.tx},
		{0,1,0,t.ty},
		{0,0,1,t.tz},
		{0,0,0,1.0}
	};
	return Matrix4(translation_matrix);
}

Matrix4 Scene::getScalingMatrix( Scaling& s)
{
	double scaling_matrix[4][4] = 
	{
		{s.sx,0,0,0},
		{0,s.sy,0,0},
		{0,0,s.sz,0},
		{0,0,0,1.0}
	};
	return Matrix4(scaling_matrix);
}

Matrix4 Scene::getRotationMatrix( Rotation& r)
{
	Vec3 u = Vec3(r.ux,r.uy,r.uz,-1);
	Vec3 v,w;
	double angle = r.angle;
	
	u=normalizeVec3(u);
	int index_of_min = getIndexOfMin(u);
	
	switch (index_of_min)
	{

	case 0:  //min is ux
		v = normalizeVec3(Vec3(0,-u.z,u.y,-1));
		break;
		
	case 1:  //min is uy
		v = normalizeVec3(Vec3(-u.z,0,u.x,-1));
		break;
		
	case 2:  //min is uz
		v = normalizeVec3(Vec3(-u.y, u.x, 0, -1));
		break;
		

	}

	w = normalizeVec3(crossProductVec3(u,v));

	double m[4][4] = 
	{
		{u.x,u.y,u.z,0},
		{v.x,v.y,v.z,0},
		{w.x,w.y,w.z,0},
		{0,0,0,1}
	};

	double radians = (angle*3.14)/180.0;

	double rot[4][4] = 
	{
		{1.0,0,0,0},
		{0,cos(radians),-1*sin(radians),0},
		{0,sin(radians),cos(radians),0},
		{0,0,0,1}
	};

	double m_reverse[4][4] = 
	{
		{u.x, v.x, w.x, 0},
		{u.y, v.y, w.y, 0},
		{u.z, v.z, w.z, 0},
		{0,0,0,1}
	};
	
	Matrix4 M = Matrix4(m);
	Matrix4 M_reverse = Matrix4(m_reverse);
	Matrix4 R = Matrix4(rot);
	return multiplyMatrixWithMatrix(  M_reverse,multiplyMatrixWithMatrix(R,M)  );
}

Matrix4 Scene::adjustModel( Mesh*& m)
{
	int t_count = m->numberOfTransformations;

	double res_init[4][4] = 
	{
		{1,0,0,0},
		{0,1,0,0},
		{0,0,1,0},
		{0,0,0,1}
	};
	Matrix4 res = Matrix4(res_init);
	Matrix4 temp ;
	for(int i=0;i<t_count;i++)
	{
		int id = m->transformationIds[i]-1;
		switch (m->transformationTypes[i])
		{
			case 't':
			{
				temp = getTranslationMatrix(*(this->translations[id]));
				res = multiplyMatrixWithMatrix(temp,res); 

				break;
			}	
			case 's':
			{
				temp = getScalingMatrix(*(this->scalings[id]));
				res = multiplyMatrixWithMatrix(temp,res); 

				break;
			}
			case 'r':
			{
				temp = getRotationMatrix(*(this->rotations[id]));
				res = multiplyMatrixWithMatrix(temp,res); 
				break;
			}
		}
	}
	return res;
}

Matrix4 Scene::adjustProjection( Mesh*& model,  Camera*& cam)
{
	Matrix4 res;
	double r,l,b,t,f,n;

	r=cam->right; l=cam->left; b=cam->bottom;
	t=cam->top;   f=cam->far;  n=cam->near;
	
	switch (cam->projectionType)
	{
		case 0:
		{
			double res_init_orth[4][4] = 
			{
				{2/(r-l), 0, 0, -1*(r+l)/(r-l)},
				{0, 2/(t-b), 0, -1*(t+b)/(t-b)},
				{0, 0, -2/(f-n), -1*(f+n)/(f-n)},
				{0, 0, 0, -1}
			};
			res = Matrix4(res_init_orth);
			break;
		}
		case 1:
		{
			double res_init_persp[4][4] = 
			{
				{2*n/(r-l), 0, (r+l)/(r-l), 0},
				{0, 2*n/(t-b), (t+b)/(t-b), 0},
				{0, 0, -1*(f+n)/(f-n), -2*(f*n)/(f-n)},
				{0, 0, -1, 0}
			};
			res = Matrix4(res_init_persp);
			break;
		}
	}
	return res;
}

Matrix4 Scene::adjustCamera( Camera*& c)
{
	
	double u_x = c->u.x; double u_y = c->u.y; double u_z = c->u.z;  
	double v_x = c->v.x; double v_y = c->v.y; double v_z = c->v.z;  
	double w_x = c->w.x; double w_y = c->w.y; double w_z = c->w.z;  

	double e_x = c->pos.x; double e_y = c->pos.y; double e_z = c->pos.z; 

	double res_init[4][4]=
	{
		{u_x, u_y, u_z, -1 * (u_x*e_x + u_y*e_y + u_z*e_z)},
		{v_x, v_y, v_z, -1 * (v_x*e_x + v_y*e_y + v_z*e_z)},
		{w_x, w_y, w_z, -1 * (w_x*e_x + w_y*e_y + w_z*e_z)},
		{0, 0, 0, 1}	
	};
	return Matrix4(res_init);
}

Matrix4 Scene::adjustViewport( Camera*& c)
{
	double n_x = c->horRes; double n_y = c->verRes;
	
	double res_init[4][4]=
	{
		{n_x/2, 0, 0, (n_x-1)/2},
		{0, n_y/2, 0, (n_y-1)/2},
		{0, 0, 0.5, 0.5},
		{0, 0, 0, 0}
	};
	return Matrix4(res_init);
	
}

/*
	Transformations, clipping, culling, rasterization are done here.
	You may define helper functions.
*/

Vec4 Scene::getThreeVertices(Triangle& t, int vertex_no)
{
	Vec4 res;
	switch (vertex_no)
	{
		case 0:
		{
			int id = t.getFirstVertexId()-1;
			double res_init[4] = 
			{
				vertices[id]->x,
				vertices[id]->y,
				vertices[id]->z,
				1,
				

			};
			res = Vec4(res_init[0], res_init[1], res_init[2], res_init[3], vertices[id]->colorId);
			break;
		}
			
		
		case 1:
		{
			int id = t.getSecondVertexId()-1;

			double res_init[4] = 
			{
				vertices[id]->x,
				vertices[id]->y,
				vertices[id]->z,
				1,
				

			};
			res = Vec4(res_init[0], res_init[1], res_init[2], res_init[3], vertices[id]->colorId);
			break;

		}

		case 2:
		{
			int id = t.getThirdVertexId()-1;
			double res_init[4] = 
			{
				vertices[id]->x,
				vertices[id]->y,
				vertices[id]->z,
				1,
				
			};
			res = Vec4(res_init[0], res_init[1], res_init[2], res_init[3], vertices[id]->colorId);
			break;
		}
	}
	return res;
		
}


double Scene::f(Vec4 v0, Vec4 v1, double x, double y)
{
	return x*(v0.y - v1.y) + y*(v1.x - v0.x) + v0.x*v1.y - v0.y*v1.x;
}


void Scene::paintLine(Vec4 v0, Vec4 v1, Camera* camera)
{

	double x_dist=v1.x-v0.x; double y_dist=v1.y-v0.y;
	
	double x0=v0.x; double x1=v1.x;
	double y0=v0.y; double y1=v1.y;	

	Color color0 = *colorsOfVertices[v0.colorId-1];
	Color color1 = *colorsOfVertices[v1.colorId-1];


	if(abs(x_dist)>abs(y_dist)) 
	{
		double min_x = max(double(0),min(x0,x1)), max_x=min(double(camera->horRes),max(x0,x1));

		double m= y_dist/x_dist;
		
		for(double x=min_x; x<max_x; x++)
		{
			double y = y0+(x-x0)*m, alpha = (x-x0)/x_dist;
			if(y>=0 && y<camera->verRes)
			{
				Color color;
			
				color.b = color0.b + alpha*(color1.b - color0.b);
				color.g = color0.g + alpha*(color1.g - color0.g);
				color.r = color0.r + alpha*(color1.r - color0.r);

				image[x][y] = Color(color);
			}	
			
		}
	} 
	else 
	{
		
		double min_y = max(double(0),min(y0,y1)), max_y=min(double(camera->verRes), max(y0,y1));

		double m= x_dist/y_dist;
		
		for(double y=min_y; y<max_y; y++)
		{
			
			
			double x = x0+(y-y0)*m, alpha = (y-y0)/y_dist;
			if(x>=0 && x<camera->horRes)
			{
				Color color;
				
				color.b = color0.b + alpha*(color1.b - color0.b);
				color.g = color0.g + alpha*(color1.g - color0.g);
				color.r = color0.r + alpha*(color1.r - color0.r);

				
				image[x][y] = Color(color);
			}
		}
	}
}


void Scene::paint(Vec4 v[3], int mesh_type, Camera* camera)
{
	switch (mesh_type)
	{
	case 0: //wireframe
	{

	
		paintLine(v[0],v[1],camera);
		paintLine(v[1],v[2],camera);
		paintLine(v[0],v[2],camera);
		
		break;
	}
	case 1: // solid
	{
		int image_x = image.size(); int image_y = image[0].size();

		double max_x=max(v[0].x, max(v[1].x,v[2].x)); double max_y=max(v[0].y, max(v[1].y,v[2].y));
		double min_x=min(v[0].x, min(v[1].x,v[2].x)); double min_y=min(v[0].y, min(v[1].y,v[2].y));


		max_x = min(max_x, double(image_x)); max_y = min(max_y, double(image_y)); 
		
		if(min_x<0){min_x = 0;}  if(min_y<0){min_y = 0;}

		double f_01, f_12, f_20, alfa, beta, gamma;
		
		for (int i=min_x; i<max_x; i++)
		{
			for (int j=min_y; j<max_y; j++)
			{
				alfa = f( v[1] , v[2] , i , j ) / f( v[1] , v[2] , v[0].x , v[0].y );

				beta = f( v[2] , v[0] , i , j ) / f( v[2] , v[0] , v[1].x , v[1].y );

				gamma = f( v[0] , v[1] , i , j ) / f( v[0] , v[1] , v[2].x , v[2].y );

				if (alfa>=0 && beta>=0 && gamma>=0)
				{

					Color color_of_vertex_0 = *colorsOfVertices[v[0].colorId - 1];
					Color color_of_vertex_1 = *colorsOfVertices[v[1].colorId - 1];
					Color color_of_vertex_2 = *colorsOfVertices[v[2].colorId - 1];

					double new_blue, new_green, new_red;

					new_blue = alfa*color_of_vertex_0.b + beta*color_of_vertex_1.b + gamma*color_of_vertex_2.b;
					new_green = alfa*color_of_vertex_0.g + beta*color_of_vertex_1.g + gamma*color_of_vertex_2.g;
					new_red = alfa*color_of_vertex_0.r + beta*color_of_vertex_1.r + gamma*color_of_vertex_2.r;
			
					this->image[i][j].b = makeBetweenZeroAnd255(new_blue);
					this->image[i][j].g = makeBetweenZeroAnd255(new_green);
					this->image[i][j].r = makeBetweenZeroAnd255(new_red);

				}
			}
		}
		break;
	}


		
	}
	
}


bool Scene::isBackfaceCulled(Vec4  v0, Vec4  v1, Vec4  v2) {
    Vec3 v_0 = Vec3(v0.x, v0.y, v0.z, v0.colorId);
    Vec3 v_1 = Vec3(v1.x, v1.y, v1.z, v1.colorId);
    Vec3 v_2 = Vec3(v2.x, v2.y, v2.z, v2.colorId);
    Vec3 edge01 = subtractVec3(v_1, v_0);
    Vec3 edge02 = subtractVec3(v_2, v_0);
    Vec3 normalVector = normalizeVec3(crossProductVec3(edge01, edge02));
    double res = dotProductVec3(normalVector, v_0); // View Vector = v_0 - origin
    return (res < 0);
}


void Scene::forwardRenderingPipeline(Camera *camera)
{
	Matrix4 cam = adjustCamera(camera);
	Matrix4 view = adjustViewport(camera);
	int max_x = image.size();
	int max_y = image[0].size();
	for(int i=0; i<meshes.size();i++)
	{
		Matrix4 model = adjustModel(meshes[i]);
		Matrix4 proj = adjustProjection(meshes[i],camera);
		int mesh_type = meshes[i]->type;
		int number_of_triangle = meshes[i]->numberOfTriangles;
		for(int j=0;j<number_of_triangle;j++)
		{
			Vec4 vertices[3];
			for(int k=0;k<3;k++)
			{
				
				Vec4 tmp1 = multiplyMatrixWithVec4(  model, getThreeVertices(meshes[i]->triangles[j], k) );
				Vec4 tmp2 = multiplyMatrixWithVec4(  cam, tmp1 );	
				Vec4 tmp3 = multiplyMatrixWithVec4(  proj, tmp2 );	

				tmp3.x /= tmp3.t;
				tmp3.y /= tmp3.t;
				tmp3.z /= tmp3.t;
				tmp3.t = 1;
				Vec4 res = multiplyMatrixWithVec4(  view, tmp3 );
				
				vertices[k]=res;

			}
			if(cullingEnabled && isBackfaceCulled(vertices[0], vertices[1], vertices[2]))
			{
				continue;
			}
			else
			{
				paint(vertices,mesh_type,camera);

			}

			
			
		}
	}
	return ;
}

/*
	Parses XML file
*/


Scene::Scene(const char *xmlPath)
{
	const char *str;
	XMLDocument xmlDoc;
	XMLElement *pElement;

	xmlDoc.LoadFile(xmlPath);

	XMLNode *pRoot = xmlDoc.FirstChild();

	// read background color
	pElement = pRoot->FirstChildElement("BackgroundColor");
	str = pElement->GetText();
	sscanf(str, "%lf %lf %lf", &backgroundColor.r, &backgroundColor.g, &backgroundColor.b);

	// read culling
	pElement = pRoot->FirstChildElement("Culling");
	if (pElement != NULL) {
		str = pElement->GetText();
		
		if (strcmp(str, "enabled") == 0) {
			cullingEnabled = true;
		}
		else {
			cullingEnabled = false;
		}
	}

	// read cameras
	pElement = pRoot->FirstChildElement("Cameras");
	XMLElement *pCamera = pElement->FirstChildElement("Camera");
	XMLElement *camElement;
	while (pCamera != NULL)
	{
		Camera *cam = new Camera();

		pCamera->QueryIntAttribute("id", &cam->cameraId);

		// read projection type
		str = pCamera->Attribute("type");

		if (strcmp(str, "orthographic") == 0) {
			cam->projectionType = 0;
		}
		else {
			cam->projectionType = 1;
		}

		camElement = pCamera->FirstChildElement("Position");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->pos.x, &cam->pos.y, &cam->pos.z);

		camElement = pCamera->FirstChildElement("Gaze");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->gaze.x, &cam->gaze.y, &cam->gaze.z);

		camElement = pCamera->FirstChildElement("Up");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->v.x, &cam->v.y, &cam->v.z);

		cam->gaze = normalizeVec3(cam->gaze);
		cam->u = crossProductVec3(cam->gaze, cam->v);
		cam->u = normalizeVec3(cam->u);

		cam->w = inverseVec3(cam->gaze);
		cam->v = crossProductVec3(cam->u, cam->gaze);
		cam->v = normalizeVec3(cam->v);

		camElement = pCamera->FirstChildElement("ImagePlane");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf %lf %lf %lf %d %d",
			   &cam->left, &cam->right, &cam->bottom, &cam->top,
			   &cam->near, &cam->far, &cam->horRes, &cam->verRes);

		camElement = pCamera->FirstChildElement("OutputName");
		str = camElement->GetText();
		cam->outputFileName = string(str);

		cameras.push_back(cam);

		pCamera = pCamera->NextSiblingElement("Camera");
	}

	// read vertices
	pElement = pRoot->FirstChildElement("Vertices");
	XMLElement *pVertex = pElement->FirstChildElement("Vertex");
	int vertexId = 1;

	while (pVertex != NULL)
	{
		Vec3 *vertex = new Vec3();
		Color *color = new Color();

		vertex->colorId = vertexId;

		str = pVertex->Attribute("position");
		sscanf(str, "%lf %lf %lf", &vertex->x, &vertex->y, &vertex->z);

		str = pVertex->Attribute("color");
		sscanf(str, "%lf %lf %lf", &color->r, &color->g, &color->b);

		vertices.push_back(vertex);
		colorsOfVertices.push_back(color);

		pVertex = pVertex->NextSiblingElement("Vertex");

		vertexId++;
	}

	// read translations
	pElement = pRoot->FirstChildElement("Translations");
	XMLElement *pTranslation = pElement->FirstChildElement("Translation");
	while (pTranslation != NULL)
	{
		Translation *translation = new Translation();

		pTranslation->QueryIntAttribute("id", &translation->translationId);

		str = pTranslation->Attribute("value");
		sscanf(str, "%lf %lf %lf", &translation->tx, &translation->ty, &translation->tz);

		translations.push_back(translation);

		pTranslation = pTranslation->NextSiblingElement("Translation");
	}

	// read scalings
	pElement = pRoot->FirstChildElement("Scalings");
	XMLElement *pScaling = pElement->FirstChildElement("Scaling");
	while (pScaling != NULL)
	{
		Scaling *scaling = new Scaling();

		pScaling->QueryIntAttribute("id", &scaling->scalingId);
		str = pScaling->Attribute("value");
		sscanf(str, "%lf %lf %lf", &scaling->sx, &scaling->sy, &scaling->sz);

		scalings.push_back(scaling);

		pScaling = pScaling->NextSiblingElement("Scaling");
	}

	// read rotations
	pElement = pRoot->FirstChildElement("Rotations");
	XMLElement *pRotation = pElement->FirstChildElement("Rotation");
	while (pRotation != NULL)
	{
		Rotation *rotation = new Rotation();

		pRotation->QueryIntAttribute("id", &rotation->rotationId);
		str = pRotation->Attribute("value");
		sscanf(str, "%lf %lf %lf %lf", &rotation->angle, &rotation->ux, &rotation->uy, &rotation->uz);

		rotations.push_back(rotation);

		pRotation = pRotation->NextSiblingElement("Rotation");
	}

	// read meshes
	pElement = pRoot->FirstChildElement("Meshes");

	XMLElement *pMesh = pElement->FirstChildElement("Mesh");
	XMLElement *meshElement;
	while (pMesh != NULL)
	{
		Mesh *mesh = new Mesh();

		pMesh->QueryIntAttribute("id", &mesh->meshId);

		// read projection type
		str = pMesh->Attribute("type");

		if (strcmp(str, "wireframe") == 0) {
			mesh->type = 0;
		}
		else {
			mesh->type = 1;
		}

		// read mesh transformations
		XMLElement *pTransformations = pMesh->FirstChildElement("Transformations");
		XMLElement *pTransformation = pTransformations->FirstChildElement("Transformation");

		while (pTransformation != NULL)
		{
			char transformationType;
			int transformationId;

			str = pTransformation->GetText();
			sscanf(str, "%c %d", &transformationType, &transformationId);

			mesh->transformationTypes.push_back(transformationType);
			mesh->transformationIds.push_back(transformationId);

			pTransformation = pTransformation->NextSiblingElement("Transformation");
		}

		mesh->numberOfTransformations = mesh->transformationIds.size();

		// read mesh faces
		char *row;
		char *clone_str;
		int v1, v2, v3;
		XMLElement *pFaces = pMesh->FirstChildElement("Faces");
        str = pFaces->GetText();
		clone_str = strdup(str);

		row = strtok(clone_str, "\n");
		while (row != NULL)
		{
			int result = sscanf(row, "%d %d %d", &v1, &v2, &v3);
			
			if (result != EOF) {
				mesh->triangles.push_back(Triangle(v1, v2, v3));
			}
			row = strtok(NULL, "\n");
		}
		mesh->numberOfTriangles = mesh->triangles.size();
		meshes.push_back(mesh);

		pMesh = pMesh->NextSiblingElement("Mesh");
	}
}

/*
	Initializes image with background color
*/
void Scene::initializeImage(Camera *camera)
{
	if (this->image.empty())
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			vector<Color> rowOfColors;

			for (int j = 0; j < camera->verRes; j++)
			{
				rowOfColors.push_back(this->backgroundColor);
			}

			this->image.push_back(rowOfColors);
		}
	}
	else
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			for (int j = 0; j < camera->verRes; j++)
			{
				this->image[i][j].r = this->backgroundColor.r;
				this->image[i][j].g = this->backgroundColor.g;
				this->image[i][j].b = this->backgroundColor.b;
			}
		}
	}
}

/*
	If given value is less than 0, converts value to 0.
	If given value is more than 255, converts value to 255.
	Otherwise returns value itself.
*/
int Scene::makeBetweenZeroAnd255(double value)
{
	if (value >= 255.0)
		return 255;
	if (value <= 0.0)
		return 0;
	return (int)(value);
}

/*
	Writes contents of image (Color**) into a PPM file.
*/
void Scene::writeImageToPPMFile(Camera *camera)
{
	ofstream fout;

	fout.open(camera->outputFileName.c_str());

	fout << "P3" << endl;
	fout << "# " << camera->outputFileName << endl;
	fout << camera->horRes << " " << camera->verRes << endl;
	fout << "255" << endl;

	for (int j = camera->verRes - 1; j >= 0; j--)
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			fout << makeBetweenZeroAnd255(this->image[i][j].r) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].g) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].b) << " ";
		}
		fout << endl;
	}
	fout.close();
}

/*
	Converts PPM image in given path to PNG file, by calling ImageMagick's 'convert' command.
	os_type == 1 		-> Ubuntu
	os_type == 2 		-> Windows
	os_type == other	-> No conversion
*/
void Scene::convertPPMToPNG(string ppmFileName, int osType)
{
	string command;

	// call command on Ubuntu
	if (osType == 1)
	{
		command = "convert " + ppmFileName + " " + ppmFileName + ".png";
		system(command.c_str());
	}

	// call command on Windows
	else if (osType == 2)
	{
		command = "magick convert " + ppmFileName + " " + ppmFileName + ".png";
		system(command.c_str());
	}

	// default action - don't do conversion
	else
	{
	}
}
