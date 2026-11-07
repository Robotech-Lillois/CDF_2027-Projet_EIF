// EUROBOT 2027 - V3 : simulateur 3D raylib, autonome, SANS Webots.
// Le modele de comportement est dans robot_core.hpp : ce fichier ne decide pas des missions.
// Compilation Debian : g++ -std=c++17 -O2 interface_3d.cpp -o eurobot -lraylib -lGL -lm -lpthread -ldl -lrt -lX11

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

// Unikement pour éviter le conflit avec le PI de robot_core.hpp
#undef PI

#include "robot_core.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace eurobot;

namespace {
constexpr int PANNEAU = 324, HAUT = 66, BAS = 58;
constexpr int TEXTURE_L = 1200, TEXTURE_H = 800;

// Utilisation de préfixes C_ pour éviter toute collision avec les macros Raylib
const Color C_NAVY={22,35,53,255}, C_PANEL={28,43,64,255}, C_CARD={39,58,80,255};
const Color C_WHITE2={239,245,250,255}, C_MUTED={170,190,207,255};
const Color C_INK={37,51,69,255}, C_BG={230,236,243,255};
const Color C_BLUE={41,138,228,255}, C_GREEN={42,179,110,255};
const Color C_YELLOW={248,190,59,255};
const Color C_BRICK={188,151,106,255};

constexpr unsigned char ARUCO_13[6][6] = {
    {1,1,1,1,1,1},
    {1,1,1,0,1,1},
    {1,0,1,0,1,1},
    {1,1,1,1,1,1},
    {1,0,0,0,0,1},
    {1,1,1,1,1,1}
};

inline Vector3 v3(float x,float y,float z){return {x,y,z};}
inline Color makeColor(int r,int g,int b,int a=255){return {static_cast<unsigned char>(r),static_cast<unsigned char>(g),static_cast<unsigned char>(b),static_cast<unsigned char>(a)};}
static unsigned hash2(int a,int b){unsigned x=static_cast<unsigned>(a)*374761393U+static_cast<unsigned>(b)*668265263U; x=(x^(x>>13))*1274126177U;return x^(x>>16);}
static bool proche(double x,double pas,double tolerance){return std::abs(std::remainder(x,pas))<tolerance;}

Color pixelSol(double x,double z,int i,int j){
    const double ax=std::abs(x),az=std::abs(z);
    const bool bleu = x>=0.0;
    const int n=static_cast<int>(hash2(i/4,j/4)%9)-4;
    Color c=makeColor(58+n,128+n,77+n);
    if (ax>0.10) {
        const double rive=.42+.36*std::pow(std::min(1.0,az/.90),1.70);
        if(ax>rive-.19 && ax<rive+.21) c=makeColor(216,122,57);
        if(ax>rive-.14 && ax<rive+.14) c=makeColor(19,93,187);
        if(ax>rive-.050 && ax<rive+.055) c=makeColor(243,243,237);
        if(ax>rive+.135 && ax<rive+.175) c=makeColor(250,144,50);
        if(ax>rive+.19) {
            const int d=static_cast<int>(hash2(i/14,j/14)%11)-5;
            c=bleu?makeColor(173+d,199+d,222+d):makeColor(233+d,189+d,101+d);
            if(proche(z,.13,.003)||proche(x+z*.08,.14,.003))
                c=bleu?makeColor(140,169,188):makeColor(200,159,95);
        }
    }
    if (ax>.72 && ax<.92 && az>.19 && az<.78) {
        const int p=static_cast<int>(hash2(i/18,j/18)%24);
        c=makeColor(177+p/3,178+p/3,175+p/3);
        if(proche(z,.095,.006)||proche(x+z*.25,.12,.006)) c=makeColor(120,128,131);
    }
    if(ax>=1.0 && ax<=1.5 && az<.25) {
        c=makeColor(135,150,165);
        if(ax<1.007||ax>1.493||az>.243)
            c=bleu?makeColor(11,105,190):makeColor(239,166,19);
    }
    if(ax>1.14 && az>.75 && az<.95) c=bleu?makeColor(85,178,231):makeColor(247,194,62);
    if(ax<.36 && az<.20) {
        c=makeColor(190,112,59);
        if(proche(x,.072,.002)||proche(z,.078,.003)) c=makeColor(124,72,45);
    }
    if(ax<.17 && az<.15) c=makeColor(66,128,74);
    return c;
}

Texture2D creerSol(){
    std::vector<Color> data(TEXTURE_L*TEXTURE_H);
    for(int j=0;j<TEXTURE_H;++j)for(int i=0;i<TEXTURE_L;++i){
        const double x=-1.5+3.0*(i+.5)/TEXTURE_L;
        const double z=-1.0+2.0*(j+.5)/TEXTURE_H;
        data[j*TEXTURE_L+i]=pixelSol(x,z,i,j);
    }
    Image image{};
    image.data=data.data();image.width=TEXTURE_L;image.height=TEXTURE_H;
    image.mipmaps=1;image.format=PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;
    Texture2D tex=LoadTextureFromImage(image);
    SetTextureFilter(tex,TEXTURE_FILTER_BILINEAR);
    return tex;
}

void cube(float x,float y,float z,float sx,float sy,float sz,Color colObj,bool contours=true){
    DrawCube(v3(x,y,z),sx,sy,sz,colObj);
    if(contours)DrawCubeWires(v3(x,y,z),sx,sy,sz,Fade(C_INK,.28f));
}

void faceTag(int axe,int signe,float sx,float sy,float sz){
    const float limite=(axe==0?std::min(sy,sz):(axe==1?std::min(sx,sz):std::min(sx,sy)));
    const float taille=std::min(.082f,limite*.80f);
    const float d=taille/6.0f;
    Vector3 origine{};
    if(axe==0)origine.x=signe*(sx*.5f+.0011f);
    if(axe==1)origine.y=signe*(sy*.5f+.0011f);
    if(axe==2)origine.z=signe*(sz*.5f+.0011f);
    Vector3 base=(axe==0?v3(.001f,taille,taille):(axe==1?v3(taille,.001f,taille):v3(taille,taille,.001f)));
    DrawCube(origine,base.x,base.y,base.z,RAYWHITE);
    for(int r=0;r<6;++r)for(int col=0;col<6;++col)if(ARUCO_13[r][col]){
        Vector3 at=origine;
        if(axe==0){at.x+=signe*.0012f;at.y+=(2.5f-r)*d;at.z+=(col-2.5f)*d;}
        if(axe==1){at.y+=signe*.0012f;at.x+=(col-2.5f)*d;at.z+=(2.5f-r)*d;}
        if(axe==2){at.z+=signe*.0012f;at.x+=(col-2.5f)*d;at.y+=(2.5f-r)*d;}
        const Vector3 s=axe==0?v3(.0012f,d+.0002f,d+.0002f):
                        (axe==1?v3(d+.0002f,.0012f,d+.0002f):v3(d+.0002f,d+.0002f,.0012f));
        DrawCube(at,s.x,s.y,s.z,BLACK);
    }
}

void pierreLocale(bool verticale){
    const float sx=verticale?.11f:.32f,sy=verticale?.32f:.11f,sz=.11f;
    cube(0,0,0,sx,sy,sz,C_BRICK);
    DrawLine3D(v3(-sx*.48f,sy*.505f,sz*.47f),v3(sx*.48f,sy*.505f,sz*.47f),makeColor(237,198,144));
    if(verticale){
        faceTag(0,1,sx,sy,sz); faceTag(0,-1,sx,sy,sz);
        faceTag(2,1,sx,sy,sz); faceTag(2,-1,sx,sy,sz);
    }else{
        faceTag(1,1,sx,sy,sz); faceTag(1,-1,sx,sy,sz);
        faceTag(2,1,sx,sy,sz); faceTag(2,-1,sx,sy,sz);
    }
}

void dessinerPierre(const Pierre& p){
    rlPushMatrix();
    rlTranslatef(static_cast<float>(p.x),static_cast<float>(p.y),static_cast<float>(p.z));
    rlRotatef(static_cast<float>(-p.yaw*180.0/PI),0,1,0);
    pierreLocale(p.verticale);
    rlPopMatrix();
}

void decorFixe(const Robot& r){
    const Color bord=makeColor(132,143,148);
    cube(0,-.069f,0,3.06f,.135f,2.06f,makeColor(111,120,123));
    cube(0,.035f,-.989f,3.02f,.070f,.022f,bord);
    cube(0,.035f,+.989f,3.02f,.070f,.022f,bord);
    cube(-1.489f,.035f,0,.022f,.070f,2.0f,bord);
    cube(+1.489f,.035f,0,.022f,.070f,2.0f,bord);
    DrawCylinder(v3(0,.021f,0),.11f,.11f,.008f,32,makeColor(210,169,68));
    DrawCylinder(v3(0,.028f,0),.065f,.065f,.005f,32,makeColor(44,116,78));
    for(const Zone& zone:r.zones){
        if(zone.type==TypeZone::Tour){
            DrawCylinder(v3((float)zone.x,.018f,(float)zone.z),.10f,.10f,.005f,28,Fade(makeColor(116,126,133),.35f));
            DrawCylinderWires(v3((float)zone.x,.018f,(float)zone.z),.10f,.10f,.006f,28,makeColor(76,90,101));
        }else{
            cube((float)zone.x,.015f,(float)zone.z,.20f,.008f,.40f,Fade(makeColor(130,140,146),.32f));
        }
    }
}

void dessinerPierres(const Robot& r){
    for(const Pierre& p:r.pierres)
        if(p.etat!=EtatPierre::Tenue)dessinerPierre(p);
}

void dessinerRobot(const Robot& r){
    const float deg=(float)(-r.pose.theta*180./PI);
    rlPushMatrix();
    rlTranslatef((float)r.pose.x,0,(float)r.pose.z);
    rlRotatef(deg,0,1,0);
    const Color chassis=r.jaune?makeColor(246,177,42):makeColor(38,116,198);
    DrawCylinder(v3(0,.005f,0),.13f,.13f,.009f,24,Fade(BLACK,.20f));
    cube(0,.09f,0,.235f,.108f,.185f,makeColor(33,51,72));
    cube(.006f,.145f,0,.220f,.014f,.170f,chassis);
    for(float side:{-1.f,+1.f}){
        DrawCylinderEx(v3(-.006f,.058f,side*.083f),v3(-.006f,.058f,side*.119f),.050f,.050f,24,makeColor(31,36,44));
        DrawCylinderEx(v3(-.006f,.058f,side*.121f),v3(-.006f,.058f,side*.124f),.028f,.028f,24,makeColor(128,139,155));
    }
    cube(.117f,.138f,0,.022f,.040f,.045f,makeColor(27,39,52));
    DrawSphere(v3(.133f,.138f,0),.012f,makeColor(111,235,247));
    DrawSphere(v3(-.060f,.160f,0),.019f,r.etat==Etat::Attente?C_YELLOW:C_GREEN);
    
    const float ext=(float)r.bras.extension, haut=(float)r.bras.hauteur;
    const Vector3 base=v3(.015f,.161f,0), articulation=v3(.095f,.264f,0);
    const Vector3 bout=v3(.165f+.18f*ext,.185f+.10f*haut,0);
    DrawCylinderEx(base,articulation,.025f,.020f,14,makeColor(68,85,105));
    DrawSphere(articulation,.029f,makeColor(40,50,61));
    DrawCylinderEx(articulation,bout,.019f,.014f,14,makeColor(57,109,169));
    DrawSphere(bout,.018f,makeColor(45,59,74));
    const float ouverture=r.bras.pinceFermee?.045f:.07f;
    cube(bout.x+.018f,bout.y,bout.z-ouverture*.5f,.068f,.014f,.012f,makeColor(80,91,106));
    cube(bout.x+.018f,bout.y,bout.z+ouverture*.5f,.068f,.014f,.012f,makeColor(80,91,106));
    if(r.bras.pierreTenue>=0){
        const bool verticale=(r.prochain<(int)r.plan.size())?r.plan[r.prochain].verticale:false;
        rlPushMatrix();
        rlTranslatef(bout.x+.090f,bout.y+(verticale?.06f:0.0f),bout.z);
        pierreLocale(verticale);
        rlPopMatrix();
    }
    DrawLine3D(v3(.13f,.017f,0),v3(.22f,.017f,0),makeColor(49,174,247));
    rlPopMatrix();
}

void dessinerAdversaire(const Robot& r){
    if(!r.adversaireVisible)return;
    rlPushMatrix();
    rlTranslatef((float)r.xAdversaire(),0,(float)r.zAdversaire());
    const Color colAdv=r.jaune?makeColor(32,131,212):makeColor(239,173,41);
    cube(0,.095f,0,.24f,.15f,.19f,colAdv);
    DrawSphere(v3(0,.18f,0),.025f,makeColor(53,60,67));
    rlPopMatrix();
}

void dessinerObstacle(const Robot& r){
    if(!r.obstacleVisible)return;
    DrawCylinder(v3((float)r.centreObstacleX(),.10f,(float)r.centreObstacleZ()),.13f,.13f,.20f,32,makeColor(195,84,84));
}

Font chargerPolice(){
    const char* candidats[]={
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
        "C:/Windows/Fonts/segoeui.ttf", "C:/Windows/Fonts/arial.ttf"
    };
    for(const char* chemin:candidats)if(FileExists(chemin)){
        Font f=LoadFontEx(chemin,40,nullptr,0);
        if(f.texture.id!=0){SetTextureFilter(f.texture,TEXTURE_FILTER_BILINEAR);return f;}
    }
    return GetFontDefault();
}

void texte(Font f,const std::string& s,float x,float y,float taille,Color colTxt){
    DrawTextEx(f,s.c_str(),{x,y},taille,1.0f,colTxt);
}

void carte(Rectangle rect,Color colCard=C_CARD){
    DrawRectangleRounded(rect,.12f,10,colCard);
}

void barre(float x,float y,float w,float progression,Color colBar){
    DrawRectangleRounded({x,y,w,8},.35f,8,makeColor(66,84,105));
    if(progression>.002f)DrawRectangleRounded({x,y,w*std::min(1.0f,progression),8},.35f,8,colBar);
}

bool bouton(Font font,Rectangle rct,const char* nom,Color accent,bool actif=true){
    const bool sur=CheckCollisionPointRec(GetMousePosition(),rct);
    carte(rct,actif?(sur?makeColor(57,77,103):C_CARD):makeColor(36,49,67));
    DrawRectangleRoundedLinesEx(rct,.12f,8,1.2f,Fade(accent,actif?.90f:.25f));
    texte(font,nom,rct.x+15,rct.y+rct.height/2-11,17,actif?C_WHITE2:C_MUTED);
    return actif&&sur&&IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

void affichagePanneau(Font font,Robot& robot,bool& pause,bool& vueDessus){
    const float w=(float)PANNEAU;
    const int h=GetScreenHeight();
    DrawRectangle(0,HAUT,PANNEAU,h-HAUT-BAS,C_PANEL);
    texte(font,"MISSION EN COURS",18,HAUT+18,13,C_MUTED);
    carte({16,(float)HAUT+43,w-32,80});
    texte(font,nomEtat(robot.etat),28,HAUT+52,17,C_WHITE2);
    texte(font,robot.message.substr(0,39),28,HAUT+79,12,C_MUTED);
    texte(font,"TEMPS",18,HAUT+136,12,C_MUTED);
    texte(font,TextFormat("%.1f / 100 s",robot.chrono),18,HAUT+153,23,C_WHITE2);
    barre(18,HAUT+188,w-36,(float)(robot.chrono/100.),C_BLUE);
    texte(font,TextFormat("Score estime : %d pts",robot.scoreEstime()),18,HAUT+201,17,C_YELLOW);
    texte(font,TextFormat("Pierres posees : %d / 16",robot.nbPosees()),18,HAUT+227,16,C_WHITE2);
    texte(font,TextFormat("Stock terrain : %d / 30",robot.nbRestantes()),18,HAUT+251,13,C_MUTED);

    texte(font,"OBJECTIF : CHATEAU",18,HAUT+280,15,C_MUTED);
    auto ligne=[&](const char* nomLigne,int fait,int cible,float yCoord,Color colLigne){
        texte(font,nomLigne,18,yCoord,15,C_WHITE2);
        texte(font,TextFormat("%d / %d",fait,cible),259,yCoord,14,colLigne);
        barre(18,yCoord+26,w-36,(float)fait/cible,colLigne);
    };
    ligne("MURS",robot.nbTerminees(TypeZone::Mur),3,HAUT+310,C_BLUE);
    ligne("TOURS",robot.nbTerminees(TypeZone::Tour),4,HAUT+352,C_YELLOW);
    ligne("PORTE",robot.nbTerminees(TypeZone::Porte),1,HAUT+394,C_GREEN);

    const float by=HAUT+447;
    if(bouton(font,{16,by,w-32,53},"CONSTRUIRE CHATEAU  [B]",C_GREEN,robot.etat==Etat::Attente)){
        robot.demarrerConstruction();pause=false;
    }
    if(bouton(font,{16,by+62,143,43},pause?"LECTURE":"PAUSE",C_BLUE))pause=!pause;
    if(bouton(font,{168,by+62,140,43},"RESET  [R]",C_YELLOW)){
        robot.reinitialiser(robot.jaune);pause=true;
    }
    if(bouton(font,{16,by+114,143,43},robot.manuel?"AUTO [M]":"MANUEL [M]",C_BLUE)){
        robot.modeManuel(!robot.manuel);pause=false;
    }
    if(bouton(font,{168,by+114,140,43},vueDessus?"ISOMETRIQUE":"VUE DESSUS",C_YELLOW))vueDessus=!vueDessus;
    if(h>920){
        texte(font,TextFormat("Position X %.2f / Z %.2f",robot.pose.x,robot.pose.z),18,by+177,13,C_MUTED);
        texte(font,TextFormat("Capteurs AV %.2f  G %.2f  D %.2f",robot.capteurs.avant,robot.capteurs.gauche,robot.capteurs.droite),18,by+199,12,C_MUTED);
        texte(font,"Saisie et pose : simulations",18,by+225,12,C_MUTED);
    }
}

void cameraScene(Camera3D& cam,bool vueDessus,float zoom,float orbit){
    cam.target=v3(0,.015f,0);
    cam.projection=CAMERA_ORTHOGRAPHIC;
    cam.fovy=(vueDessus?3.10f:3.72f)*zoom;
    cam.up=vueDessus?v3(0,0,-1):v3(0,1,0);
    if(vueDessus)cam.position=v3(.001f,5.2f,.001f);
    else cam.position=v3(4.0f*(float)std::cos(orbit),3.7f,4.0f*(float)std::sin(orbit));
}
} // namespace

int main(){
    SetConfigFlags(FLAG_WINDOW_RESIZABLE|FLAG_MSAA_4X_HINT);
    InitWindow(1450,890,"EUROBOT 2027 | Constructeur de chateau | C++ raylib");
    SetWindowMinSize(1120,760);SetTargetFPS(60);
    Font font=chargerPolice();
    Texture2D sol=creerSol();
    Mesh mesh=GenMeshPlane(3.0f,2.0f,1,1);
    Material mat=LoadMaterialDefault();
    SetMaterialTexture(&mat,MATERIAL_MAP_DIFFUSE,sol);
    Robot robot;
    bool pause=true,dessus=false,traceVisible=true;
    float zoom=1.0f,orbit=.75f;
    const double dt=1.0/120.0;
    double cumul=0.0;
    std::vector<Vector3> parcours;
    int vw=GetScreenWidth()-PANNEAU,vh=GetScreenHeight()-HAUT-BAS;
    RenderTexture2D cible=LoadRenderTexture(vw,vh);
    Camera3D cam{};
    while(!WindowShouldClose()){
        const int W=GetScreenWidth(),H=GetScreenHeight();
        const int newW=W-PANNEAU,newH=H-HAUT-BAS;
        if(newW!=vw||newH!=vh){
            UnloadRenderTexture(cible);
            vw=newW;vh=newH;
            cible=LoadRenderTexture(vw,vh);
        }
        if(IsKeyPressed(KEY_B)) {robot.demarrerConstruction();pause=false;}
        if(IsKeyPressed(KEY_SPACE))pause=!pause;
        if(IsKeyPressed(KEY_R)){robot.reinitialiser(robot.jaune);pause=true;parcours.clear();cumul=0;}
        if(IsKeyPressed(KEY_V))dessus=!dessus;
        if(IsKeyPressed(KEY_M)){robot.modeManuel(!robot.manuel);pause=false;}
        if(IsKeyPressed(KEY_T))traceVisible=!traceVisible;
        if(IsKeyPressed(KEY_O))robot.obstacleVisible=!robot.obstacleVisible;
        if(IsKeyPressed(KEY_P))robot.adversaireVisible=!robot.adversaireVisible;
        if(IsKeyPressed(KEY_C)&&robot.etat==Etat::Attente){robot.reinitialiser(!robot.jaune);parcours.clear();pause=true;}
        const Vector2 souris=GetMousePosition();
        if(souris.x>PANNEAU){
            zoom=(float)borne(zoom-GetMouseWheelMove()*.045,.72,1.38);
            if(IsMouseButtonDown(MOUSE_BUTTON_LEFT)&&!dessus){orbit+=(GetMouseDelta().x*.004f);}
        }
        cameraScene(cam,dessus,zoom,orbit);
        if(!pause){
            cumul+=std::min(.065,(double)GetFrameTime());
            int garde=0;
            while(cumul>=dt&&garde++<16){
                double avancer=0,tourner=0;
                if(robot.manuel){
                    if(IsKeyDown(KEY_UP)||IsKeyDown(KEY_W))avancer+=1;
                    if(IsKeyDown(KEY_DOWN)||IsKeyDown(KEY_S))avancer-=1;
                    if(IsKeyDown(KEY_LEFT)||IsKeyDown(KEY_A))tourner+=1;
                    if(IsKeyDown(KEY_RIGHT)||IsKeyDown(KEY_D))tourner-=1;
                }
                robot.avancer(dt,avancer,tourner);
                const Vector3 maintenant=v3((float)robot.pose.x,.027f,(float)robot.pose.z);
                if(parcours.empty()||Vector3Distance(maintenant,parcours.back())>.015f){
                    parcours.push_back(maintenant);
                    if(parcours.size()>2400)parcours.erase(parcours.begin());
                }
                cumul-=dt;
                if(robot.etat==Etat::FinTemps||robot.etat==Etat::Termine){pause=true;break;}
            }
        }
        BeginTextureMode(cible);
        ClearBackground(C_BG);
        BeginMode3D(cam);
        decorFixe(robot);
        DrawMesh(mesh,mat,MatrixTranslate(0,.004f,0));
        dessinerPierres(robot);
        dessinerObstacle(robot);
        dessinerAdversaire(robot);
        if(traceVisible){
            for(size_t k=1;k<parcours.size();++k)
                DrawLine3D(parcours[k-1],parcours[k],makeColor(59,170,240));
            if(robot.etat==Etat::VersPierre||robot.etat==Etat::VersChateau||robot.etat==Etat::RetourBase){
                const Pose objectif=robot.destination();
                DrawLine3D(v3((float)robot.pose.x,.035f,(float)robot.pose.z),
                    v3((float)objectif.x,.035f,(float)objectif.z),Fade(C_BLUE,.7f));
            }
        }
        dessinerRobot(robot);
        EndMode3D();EndTextureMode();

        BeginDrawing();
        ClearBackground(C_NAVY);
        DrawRectangle(0,0,W,HAUT,C_NAVY);
        DrawRectangle(0,0,7,HAUT,C_BLUE);
        texte(font,"EUROBOT 2027",22,13,29,C_WHITE2);
        texte(font,"SIMULATEUR 3D C++  |  CONSTRUCTION AUTOMATIQUE",350,21,17,C_MUTED);
        DrawRectangle(PANNEAU,HAUT,vw,vh,C_BG);
        DrawTexturePro(cible.texture,{0,0,(float)cible.texture.width,-(float)cible.texture.height},
            {(float)PANNEAU,(float)HAUT,(float)vw,(float)vh},{0,0},0,WHITE);
        carte({(float)PANNEAU+16,(float)HAUT+16,277,65},Fade(RAYWHITE,.93f));
        texte(font,"TABLE EUROBOT 2027",PANNEAU+29,HAUT+25,19,C_INK);
        texte(font,dessus?"VUE DU DESSUS  |  3 x 2 m":"VUE ISOMETRIQUE  |  3 x 2 m",PANNEAU+29,HAUT+52,13,makeColor(95,109,123));
        affichagePanneau(font,robot,pause,dessus);
        DrawRectangle(0,H-BAS,W,BAS,C_NAVY);
        texte(font,"ESPACE Pause  |  B Chateau  |  M Manuel  |  Fleches Bouger  |  R Reset",19,H-BAS+11,15,C_WHITE2);
        texte(font,"V Vue  |  T Trace  |  O Obstacle  |  P Adversaire  |  C Equipe",19,H-BAS+33,13,C_MUTED);
        texte(font,TextFormat("%d FPS",GetFPS()),W-80,H-BAS+19,17,C_GREEN);
        EndDrawing();
        if(robot.chrono==0.0 && robot.etat==Etat::Attente && !parcours.empty())parcours.clear();
    }
    UnloadRenderTexture(cible);
    UnloadMesh(mesh);
    UnloadTexture(sol);
    if(font.texture.id!=GetFontDefault().texture.id)UnloadFont(font);
    CloseWindow();
    return 0;
}
