#pragma once
// EUROBOT 2027 - V4 (C++17) : coeur de simulation independant de raylib.
// Les actions de pince, les capteurs et la physique sont des MODELES SIMPLIFIES.
// Un robot physique necessitera des pilotes materiels, une calibration et des essais de securite.

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <queue>
#include <limits>
#include <functional>

namespace eurobot {
constexpr double PI = 3.14159265358979323846;
constexpr double DUREE_MATCH = 100.0;
inline double borne(double x, double lo, double hi) { return std::max(lo, std::min(x, hi)); }
inline double angleNormalise(double x) {
    while (x > PI) x -= 2.0 * PI;
    while (x < -PI) x += 2.0 * PI;
    return x;
}

struct Pose { double x = 1.24, z = 0.0, theta = PI; };
struct Roues { double gauche = 0.0, droite = 0.0; };
struct Distances { double avant = 2.0, gauche = 2.0, droite = 2.0; };

enum class Etat { Attente, VersPierre, Saisir, VersChateau, Deposer, RetourBase, Termine, FinTemps };
enum class EtatPierre { Carriere, Tenue, Construite };
enum class TypeZone { Mur, Tour, Porte };

inline const char* nomEtat(Etat e) {
    switch (e) {
        case Etat::Attente: return "EN ATTENTE";
        case Etat::VersPierre: return "CHERCHE UNE PIERRE";
        case Etat::Saisir: return "PRISE AVEC LE BRAS";
        case Etat::VersChateau: return "TRANSPORT DE PIERRE";
        case Etat::Deposer: return "CONSTRUCTION";
        case Etat::RetourBase: return "RETOUR AU TRONE";
        case Etat::Termine: return "CHATEAU / RETOUR TERMINE";
        default: return "100 S : ARRET";
    }
}
inline const char* nomType(TypeZone t) {
    switch (t) {
        case TypeZone::Mur: return "MUR";
        case TypeZone::Tour: return "TOUR";
        default: return "PORTE";
    }
}

// Une pierre est un carton 320 x 110 x 110 mm, avec ArUco ID 13 (jeu 2027).
struct Pierre {
    double x = 0, y = 0.055, z = 0, yaw = 0;
    bool verticale = false;
    EtatPierre etat = EtatPierre::Carriere;
    int carriere = -1, niveau = 0, emplacement = -1;
};
struct Carriere {
    double x = 0, z = 0, yaw = 0, approcheAngle = 0;
    int cote = 0;         // -1 = jaune, +1 = bleu, 0 = partagee
    int restantes = 3;  // trois cartons empiles au depart
};
struct Zone {
    TypeZone type = TypeZone::Mur;
    double x = 0, z = 0;
    int requis = 0, poses = 0;
};
struct Emplacement {
    double x = 0, y = 0, z = 0, yaw = 0;
    bool verticale = false;
    int zone = -1;
    double approcheAngle = PI; // orientation du robot au point de pose
    int pierre = -1;
};
struct Bras {
    double extension = 0.0;  // 0 replie, 1 deploye
    double hauteur = 0.0;    // 0 bas, 1 haut (indicatif)
    bool pinceFermee = false;
    int pierreTenue = -1;
};

class Robot {
public:
    // Hypotheses DE SIMULATION : ne sont pas des specifications du robot reel.
    static constexpr double entraxe = 0.18;
    static constexpr double vitesseRoueMax = 1.65; // m/s, acceleree pour demo 100 s
    static constexpr double vitesseCroisiere = 1.35;
    static constexpr double omegaMax = 6.5;
    static constexpr double porteeBras = 0.34;
    static constexpr double rayonRobot = 0.12;

    Pose pose{};
    Pose depart{};
    Etat etat = Etat::Attente;
    Bras bras{};
    Distances capteurs{};
    double chrono = 0.0, distance = 0.0;
    bool eviter = true;
    bool manuel = false;
    bool obstacleVisible = false;
    bool adversaireVisible = false;
    bool jaune = false;
    double adversairePhase = 0.0;
    std::vector<Carriere> carrieres;
    std::vector<Pierre> pierres;
    std::vector<Zone> zones;
    std::vector<Emplacement> plan;
    int prochain = 0;
    std::string message = "Appuyer sur CONSTRUIRE CHATEAU (B)";

    Robot() { reinitialiser(); }

    void reinitialiser(bool equipeJaune = false) {
        jaune = equipeJaune;
        const double m = jaune ? -1.0 : 1.0;
        depart = {m * 1.24, 0.0, jaune ? 0.0 : PI};
        pose = depart; etat = Etat::Attente; bras = {}; capteurs = {};
        chrono = distance = adversairePhase = 0.0;
        eviter = true; manuel = false; obstacleVisible = adversaireVisible = false;
        prochain = 0; tempsBras = 0.0; carriereChoisie = pierreChoisie = -1;
        message = "Appuyer sur CONSTRUIRE CHATEAU (B)";
        initialiserTerrain();
    }

    void demarrerConstruction() {
        if (etat != Etat::Attente && etat != Etat::RetourBase) return;
        manuel = false;
        planifier();
    }
    void modeManuel(bool actif) {
        if (etat == Etat::FinTemps || etat == Etat::Termine) return;
        manuel = actif;
        if (manuel) message = "Mode manuel : fleches du clavier";
        else message = "Mode automatique";
    }
    int nbPosees() const {
        int n = 0;
        for (const Pierre& p : pierres) if (p.etat == EtatPierre::Construite) ++n;
        return n;
    }
    int nbRestantes() const {
        int n = 0;
        for (const Pierre& p : pierres) if (p.etat == EtatPierre::Carriere) ++n;
        return n;
    }
    int nbTerminees(TypeZone type) const {
        int n = 0;
        for (const Zone& z : zones) if (z.type == type && z.poses == z.requis) ++n;
        return n;
    }
    bool chateauComplet() const { return prochain == static_cast<int>(plan.size()); }
    void recalculerRoute() { if (etat==Etat::VersPierre || etat==Etat::VersChateau || etat==Etat::RetourBase) tracerRoute(); }
    bool dansTrone() const {
        return std::abs(pose.x - depart.x) <= (0.25 - rayonRobot) &&
               std::abs(pose.z - depart.z) <= (0.25 - rayonRobot);
    }
    int scoreEstime() const {
        int s = nbPosees(); // 1 point / pierre construite
        for (const Zone& z : zones)
            if (z.poses == z.requis) s += (z.type == TypeZone::Porte ? 20 : 5);
        if ((etat == Etat::Termine || etat == Etat::FinTemps) && dansTrone()) s += 10;
        return s;
    }
    // Point de navigation vise, utile pour afficher une trajectoire dans l'interface.
    Pose destination() const {
        if (etat == Etat::VersPierre && carriereChoisie >= 0) {
            const Carriere& q = carrieres[carriereChoisie];
            return pointApproche(q.x, q.z, q.approcheAngle);
        }
        if (etat == Etat::VersChateau && prochain < static_cast<int>(plan.size())) {
            const Emplacement& e = plan[prochain];
            return pointApproche(e.x, e.z, e.approcheAngle);
        }
        return depart;
    }
    // Mesures de distance TRES simplifiees de 3 rayons a l'avant du robot.
    Distances mesures() const {
        Distances d;
        const double angles[3] = {0.0, PI / 3.0, -PI / 3.0};
        double* dest[3] = {&d.avant, &d.gauche, &d.droite};
        for (int k = 0; k < 3; ++k) {
            const double a = pose.theta + angles[k];
            const double ux = std::cos(a), uz = std::sin(a);
            for (double t = 0.025; t < 1.8; t += 0.035) {
                const double x = pose.x + ux * t, z = pose.z + uz * t;
                bool touche = std::abs(x) > 1.49 || std::abs(z) > 0.99;
                if (obstacleVisible && std::hypot(x - centreObstacleX(), z - centreObstacleZ()) < .145)
                    touche = true;
                if (adversaireVisible) {
                    const double ax = xAdversaire(), az = zAdversaire();
                    if (std::hypot(x-ax, z-az) < 0.16) touche = true;
                }
                if (touche) { *dest[k] = std::max(0.0, t - rayonRobot); break; }
            }
        }
        return d;
    }
    double centreObstacleX() const { return jaune ? -0.30 : 0.30; }
    double centreObstacleZ() const { return 0.16; }
    double xAdversaire() const { return jaune ? +0.85 : -0.85; }
    double zAdversaire() const { return 0.43 * std::sin(adversairePhase); }

    // Commandes roue gauche/droite, compatibles avec une future couche moteurs.
    Roues commande(const Pose& courante, const Pose& cible, double tolerance, const Distances& d) const {
        double dx = cible.x - courante.x, dz = cible.z - courante.z;
        double rho = std::hypot(dx, dz), v = 0.0, w = 0.0;
        if (rho > tolerance) {
            const double er = angleNormalise(std::atan2(dz, dx) - courante.theta);
            v = borne(4.8 * rho, 0.0, vitesseCroisiere) *
                borne(1.0 - std::abs(er) / 1.1, 0.0, 1.0);
            w = borne(6.0 * er, -omegaMax, omegaMax);
            // On ne tente pas de "reconnaitre" les pierres via cette mesure :
            // elles sont accessibles par le plan de carriere connu.
            if (eviter && (obstacleVisible || adversaireVisible) && d.avant < .16) {
                v = 0.0; w = (d.gauche >= d.droite ? 2.4 : -2.4);
            }
        } else {
            const double er = angleNormalise(cible.theta - courante.theta);
            if (std::abs(er) < .08) return {};
            w = borne(6.0 * er, -omegaMax, omegaMax);
        }
        return {borne(v - w * entraxe * .5, -vitesseRoueMax, vitesseRoueMax),
                borne(v + w * entraxe * .5, -vitesseRoueMax, vitesseRoueMax)};
    }

    // Kinematique diff-drive de simulation ; pas un controle moteur reel.
    void simulerMouvement(const Roues& c, double dt) {
        const double v = (c.gauche + c.droite) * 0.5;
        const double w = (c.droite - c.gauche) / entraxe;
        const double angleMilieu = pose.theta + w * dt * 0.5;
        const double x = pose.x + v * std::cos(angleMilieu) * dt;
        const double z = pose.z + v * std::sin(angleMilieu) * dt;
        // Sol et bordures : le centre du robot ne doit pas franchir le plateau.
        if (std::abs(x) < 1.5 - rayonRobot && std::abs(z) < 1.0 - rayonRobot) {
            pose.x = x; pose.z = z;
            distance += std::abs(v * dt);
        }
        pose.theta = angleNormalise(pose.theta + w * dt);
    }

    // Appeler avec dt fixe (par ex. 1/120 s) lorsque la simulation est en lecture.
    void avancer(double dt, double manuelAvance = 0.0, double manuelTourne = 0.0) {
        if (etat == Etat::FinTemps || etat == Etat::Termine) return;
        dt = borne(dt, 0.0, .05);
        if (etat == Etat::Attente && !manuel) return;
        chrono += dt;
        adversairePhase += dt * .7;
        capteurs = mesures();
        if (chrono >= DUREE_MATCH) {
            chrono = DUREE_MATCH; etat = Etat::FinTemps;
            message = "Arret reglementaire apres 100 s";
            return;
        }
        if (manuel) {
            const double v = manuelAvance * .48;
            const double w = manuelTourne * 3.0;
            simulerMouvement({v - .5 * entraxe * w, v + .5 * entraxe * w}, dt);
            return;
        }
        switch (etat) {
            case Etat::VersPierre:
                if (deplacementTermine(destination(), dt)) {
                    etat = Etat::Saisir; tempsBras = 0;
                    message = "Bras deploye : saisie du carton";
                }
                break;
            case Etat::Saisir: {
                tempsBras += dt;
                bras.extension = trapeze(tempsBras);
                bras.hauteur = borne(tempsBras / .65, 0.0, 1.0);
                if (tempsBras >= .43 && bras.pierreTenue < 0) {
                    Pierre& s = pierres[pierreChoisie];
                    s.etat = EtatPierre::Tenue;
                    --carrieres[carriereChoisie].restantes;
                    bras.pierreTenue = pierreChoisie;
                    bras.pinceFermee = true;
                    message = "Pierre recuperee et transportee";
                }
                if (tempsBras >= .85) {
                    bras.extension = 0; bras.hauteur = 1;
                    etat = Etat::VersChateau;
                    tracerRoute();
                }
                break;
            }
            case Etat::VersChateau:
                if (deplacementTermine(destination(), dt)) {
                    etat = Etat::Deposer; tempsBras = 0;
                    message = "Pose de la pierre dans le chateau";
                }
                break;
            case Etat::Deposer: {
                tempsBras += dt;
                bras.extension = trapeze(tempsBras);
                bras.hauteur = 1.0 - borne(tempsBras / .70, 0.0, 1.0);
                if (tempsBras >= .43 && bras.pierreTenue >= 0) {
                    Pierre& p = pierres[bras.pierreTenue];
                    const Emplacement& e = plan[prochain];
                    p.etat = EtatPierre::Construite;
                    p.x = e.x; p.z = e.z; p.y = e.y; p.yaw = e.yaw;
                    p.verticale = e.verticale; p.emplacement = prochain;
                    ++zones[e.zone].poses;
                    bras.pierreTenue = -1; bras.pinceFermee = false;
                    message = std::string(nomType(zones[e.zone].type)) + " : pierre posee";
                    ++prochain;
                }
                if (tempsBras >= .85) { bras.extension = 0; bras.hauteur = 0; planifier(); }
                break;
            }
            case Etat::RetourBase:
                if (deplacementTermine(depart, dt)) {
                    etat = Etat::Termine;
                    message = chateauComplet() ? "Chateau termine ! Retour reussi" : "Retour effectue avant 100 s";
                }
                break;
            default: break;
        }
    }

private:
    int carriereChoisie = -1, pierreChoisie = -1;
    double tempsBras = 0.0;
    std::vector<Pose> route;
    std::size_t etapeRoute = 0;

    static double trapeze(double t) {
        return borne(t / .30, 0.0, 1.0) - borne((t - .50) / .30, 0.0, 1.0);
    }
    static Pose pointApproche(double x, double z, double angle) {
        return {x - porteeBras * std::cos(angle), z - porteeBras * std::sin(angle), angle};
    }
    bool deplacementTermine(const Pose& but, double dt) {
        // Etapes intermediaires d'une route A*, lisses via visibilite directe.
        while (etapeRoute < route.size()) {
            const Pose& p = route[etapeRoute];
            if (std::hypot(p.x - pose.x, p.z - pose.z) < .095) { ++etapeRoute; continue; }
            simulerMouvement(commande(pose, p, .075, capteurs), dt);
            return false;
        }
        const double dist = std::hypot(but.x - pose.x, but.z - pose.z);
        const double er = std::abs(angleNormalise(but.theta - pose.theta));
        if (dist < .047 && er < .10) return true;
        simulerMouvement(commande(pose, but, .047, capteurs), dt);
        return false;
    }
    bool occupe(double x,double z) const {
        constexpr double M=.125; // encombrement robot + marge virtuelle
        if(std::abs(x)>1.50-M || std::abs(z)>1.0-M) return true;
        for(const Pierre& p:pierres){
            if(p.etat==EtatPierre::Tenue)continue;
            const double dx=x-p.x,dz=z-p.z,c=std::cos(p.yaw),s=std::sin(p.yaw);
            const double lx=dx*c+dz*s,lz=-dx*s+dz*c;
            const double hx=p.verticale?.055:.160;
            if(std::abs(lx)<hx+M && std::abs(lz)<.055+M)return true;
        }
        if(obstacleVisible && std::hypot(x-centreObstacleX(),z-centreObstacleZ())<.15+M)return true;
        if(adversaireVisible && std::hypot(x-xAdversaire(),z-zAdversaire())<.16+M)return true;
        return false;
    }
    bool ligneLibre(double x1,double z1,double x2,double z2) const {
        double dist=std::hypot(x2-x1,z2-z1);
        int n=std::max(1,static_cast<int>(std::ceil(dist/.027)));
        for(int i=0;i<=n;++i){
            double a=static_cast<double>(i)/n;
            if(occupe(x1+(x2-x1)*a,z1+(z2-z1)*a))return false;
        }
        return true;
    }
    void tracerRoute() {
        route.clear();etapeRoute=0;
        const Pose but=destination();
        if(ligneLibre(pose.x,pose.z,but.x,but.z)) return;
        constexpr double STEP=.05;
        constexpr int NX=55,NZ=35;
        auto index=[&](int x,int z){return x+z*NX;};
        auto xGrille=[&](int ix){return -1.35+ix*STEP;};
        auto zGrille=[&](int iz){return -.85+iz*STEP;};
        auto nearest=[&](double x,double z){
            int ix=static_cast<int>(borne(std::round((x+1.35)/STEP),0,NX-1));
            int iz=static_cast<int>(borne(std::round((z+.85)/STEP),0,NZ-1));
            // Plusieurs cellules candidates autour du but si l'une est occupee.
            int best=index(ix,iz); double cost=1e12;
            for(int dz=-5;dz<=5;++dz)for(int dx=-5;dx<=5;++dx){
                int xx=ix+dx,zz=iz+dz;
                if(xx<0||xx>=NX||zz<0||zz>=NZ||occupe(xGrille(xx),zGrille(zz)))continue;
                const double d=std::hypot(xGrille(xx)-x,zGrille(zz)-z);
                if(d<cost){cost=d;best=index(xx,zz);}
            }
            return best;
        };
        const int start=nearest(pose.x,pose.z),goal=nearest(but.x,but.z);
        const int nn=NX*NZ;
        std::vector<double> g(nn,std::numeric_limits<double>::infinity());
        std::vector<int> prev(nn,-1);
        std::vector<bool> closed(nn,false);
        using Noeud=std::pair<double,int>;
        std::priority_queue<Noeud,std::vector<Noeud>,std::greater<Noeud>> ouverts;
        g[start]=0; ouverts.push({0,start});
        const int voisins[8][2]={{1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,1},{1,-1},{-1,-1}};
        int boucles=0;
        while(!ouverts.empty() && boucles++<nn*6){
            const int a=ouverts.top().second;ouverts.pop();
            if(closed[a])continue;
            if(a==goal)break;
            closed[a]=true;
            int ix=a%NX, iz=a/NX;
            for(auto& v:voisins){
                const int nx=ix+v[0],nz=iz+v[1];
                if(nx<0||nx>=NX||nz<0||nz>=NZ)continue;
                if(occupe(xGrille(nx),zGrille(nz)))continue;
                // Refuser de couper un angle occupe par une pierre.
                if(v[0] && v[1] && (occupe(xGrille(ix+v[0]),zGrille(iz)) ||
                                     occupe(xGrille(ix),zGrille(iz+v[1]))))continue;
                int b=index(nx,nz);
                double ng=g[a]+(v[0]&&v[1]?1.414213562:1.0);
                if(ng<g[b]){
                    g[b]=ng;prev[b]=a;
                    double h=std::hypot((nx-goal%NX)*1.0,(nz-goal/NX)*1.0);
                    ouverts.push({ng+h,b});
                }
            }
        }
        if(start!=goal && prev[goal]<0) return; // pas de route connue ; commande directe secours
        std::vector<Pose> brute;
        int u=goal;
        while(u!=start && u>=0){brute.push_back({xGrille(u%NX),zGrille(u/NX),0});u=prev[u];}
        std::reverse(brute.begin(),brute.end());
        // Lissage de route : garder uniquement les points dont le segment est degage.
        double sx=pose.x,sz=pose.z;
        std::size_t i=0;
        while(i<brute.size()){
            std::size_t j=i;
            for(std::size_t k=brute.size();k>i;--k){
                if(ligneLibre(sx,sz,brute[k-1].x,brute[k-1].z)){j=k-1;break;}
            }
            route.push_back(brute[j]);
            sx=brute[j].x;sz=brute[j].z;i=j+1;
        }
    }
    void planifier() {
        if (prochain >= static_cast<int>(plan.size())) {
            etat = Etat::RetourBase;
            message = "19 pierres posees : retour salle du trone";
            tracerRoute();
            return;
        }
        // Repartir seulement si le budget de temps estime laisse une marge de retour.
        // Estimation approximative : elle n'est pas une garantie de resultat en vrai match.
        const Emplacement& e = plan[prochain];
        const Pose arrivee = pointApproche(e.x, e.z, e.approcheAngle);
        int choix = -1;
        double coutMin = 1e8, coutTrajet = 0;
        for (size_t i = 0; i < carrieres.size(); ++i) {
            const Carriere& q = carrieres[i];
            if (q.restantes <= 0) continue; // Pierres accessibles dans ce scenario simplifie.
            if (prochain < 6 && q.cote != 0) continue;  // retirer les piles centrales avant de fermer les remparts
            const Pose retrait = pointApproche(q.x, q.z, q.approcheAngle);
            const double cout = std::hypot(pose.x - retrait.x, pose.z - retrait.z) +
                                std::hypot(retrait.x - arrivee.x, retrait.z - arrivee.z);
            if (cout < coutMin) { coutMin = cout; coutTrajet = cout; choix = static_cast<int>(i); }
        }
        if (choix < 0) { etat = Etat::RetourBase; message = "Plus de pierres accessibles"; tracerRoute(); return; }
        // Delai : distance/vitesse + prises/deposes + marge d'orientation + retour base.
        const double ret = std::hypot(arrivee.x-depart.x, arrivee.z-depart.z)/vitesseCroisiere;
        const double estimation = coutTrajet/vitesseCroisiere + 1.2 + ret + 1.2;
        if (chrono + estimation >= DUREE_MATCH) {
            etat = Etat::RetourBase;
            message = "Temps restant : retour au trone";
            tracerRoute();
            return;
        }
        carriereChoisie = choix;
        const Carriere& q = carrieres[choix];
        pierreChoisie = -1;
        for (size_t k = 0; k < pierres.size(); ++k) {
            const Pierre& s = pierres[k];
            if (s.carriere == choix && s.etat == EtatPierre::Carriere && s.niveau == q.restantes-1)
                pierreChoisie = static_cast<int>(k);
        }
        if (pierreChoisie < 0) { etat = Etat::RetourBase; message = "Stock introuvable"; tracerRoute(); return; }
        etat = Etat::VersPierre;
        tracerRoute();
        message = "Vers carriere : prendre pierre " + std::to_string(prochain + 1) + "/" + std::to_string(plan.size());
    }
    void initialiserTerrain() {
        carrieres.clear(); pierres.clear(); zones.clear(); plan.clear();
        const double mirror = jaune ? -1.0 : +1.0;
        // 10 carrieres de 3 pierres = 30 cartons, repartis en bordure et au centre.
        // Positions ILLUSTRATIVES, a recaler sur le plan annexe officiel (Fig. 8).
        auto creerCarriere = [&](double x, double z, int cote, double angle) {
            Carriere q;
            q.x = mirror*x; q.z = z; q.cote = cote==0 ? 0 : (jaune ? -cote : cote);
            q.yaw = 0; q.approcheAngle = jaune ? angleNormalise(PI - angle) : angle;
            const int id = static_cast<int>(carrieres.size());
            carrieres.push_back(q);
            for (int niveau = 0; niveau < 3; ++niveau) {
                Pierre p; p.x = q.x; p.z = q.z; p.y = .055 + .11*niveau;
                p.carriere = id; p.niveau = niveau;
                pierres.push_back(p);
            }
        };
        for (int side : {-1,+1}) {
            const double z = .86 * side;
            const double approche = (side == +1 ? PI/2.0 : -PI/2.0);
            creerCarriere(-1.12, z, -1, approche);
            creerCarriere(-.39,  z, -1, approche);
            creerCarriere( .39,  z, +1, approche);
            creerCarriere( 1.12, z, +1, approche);
            creerCarriere(0.0, side*.42, 0, PI); // prise depuis la partie bleue
        }
        // Objectif : 4 murs + 4 tours + 1 porte = 19 pierres.
        // Seuls les EMPLACEMENTS sont dessines au debut, aucun mur preconstruit.
        auto zone = [&](TypeZone type, double x, double z, int requis) {
            const int id = static_cast<int>(zones.size());
            zones.push_back({type, mirror*x, z, requis, 0});
            return id;
        };
        auto slot = [&](int id, double x, double y, double z, double yaw, bool vert) {
            plan.push_back({mirror*x, y, z, jaune ? angleNormalise(PI-yaw) : yaw,
                            vert, id, (x >= 1.04 ? (jaune ? PI : 0.0) : (jaune ? 0.0 : PI)), -1});
        };
        const int porte = zone(TypeZone::Porte, .69, 0.0, 3);
        slot(porte, .69, .16, -.145, 0.0, true);
        slot(porte, .69, .16, +.145, 0.0, true);
        slot(porte, .69, .375, 0.0, PI/2, false);
        const double tourXY[4][2] = {{.82,-.67},{.82,+.67},{1.13,-.73},{1.13,+.73}};
        for (const auto& p : tourXY) {
            const int id = zone(TypeZone::Tour, p[0], p[1], 1);
            slot(id, p[0], .16, p[1], 0.0, true);
            if (p[0] > 1.04) plan.back().approcheAngle = (p[1] > 0 ? PI/2 : -PI/2);
        }
        // Deux murs sur la partie interne et deux murs lateraux symetriques.
        const double murXY[4][2] = {{.72,-.38},{.72,+.38},{1.10,-.35},{1.10,+.35}};
        for (const auto& p : murXY) {
            const int id = zone(TypeZone::Mur, p[0], p[1], 3);
            for (int k = 0; k < 3; ++k) {
                slot(id, p[0], .055+.11*k, p[1], PI/2, false);
                if (p[0] > 1.04) plan.back().approcheAngle = (p[1] > 0 ? PI/2 : -PI/2);
            }
        }
    }
};
} // namespace eurobot
