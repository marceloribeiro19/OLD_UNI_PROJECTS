#include <list>
#include <PDCurses-master/curses.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>

using namespace std;

template <typename T>
bool compare(const T& a,const  T& b)
{
    return a > b;
}

class Score
{
private:
    int score;
    string name;
public:
    Score()
    {
        score = 0;
        name = "";
    }
    Score(int n, string str)
    {
        score = n;
        name = str;
    }
    bool operator>(const Score& snd)const
    {
        return score > snd.score;
    }
    ~Score()
    {
        name.erase();
    }
    int get_score() { return score; }
    string get_name() { return name; }

};

class snake {
private:
    int x, y;
public:
    snake(int a, int b) {
        x = a;
        y = b;
    }
    int getX() { return x; }
    int getY() { return y; }
};

void print_snake()
{
    init_pair(5, COLOR_GREEN, NULL);

    attron(COLOR_PAIR(5));

    mvprintw(2, 27, "                     ,--.                      ,--.           ");
    mvprintw(3, 27, "  .--.--.          ,--.'|   ,---,          ,--/  /|    ,---,. ");
    mvprintw(4, 27, " /  /    '.    ,--,:  : |  '  .' \\      ,---,': / '  ,'  .' | ");
    mvprintw(5, 27, "|  :  /`. / ,`--.'`|  ' : /  ;    '.    :   : '/ / ,---.'   | ");
    mvprintw(6, 27, ";  |  |--`  |   :  :  | |:  :       \\   |   '   ,  |   |   .' ");
    mvprintw(7, 27, "|  :  ;_    :   |   \\ | ::  |   /\\   \\  '   |  /   :   :  |-, ");
    mvprintw(8, 27, " \\  \\    `. |   : '  '; ||  :  ' ;.   : |   ;  ;   :   |  ;/| ");
    mvprintw(9, 27, "  `----.   \\'   ' ;.    ;|  |  ;/  \\   \\:   '   \\  |   :   .' ");
    mvprintw(10, 27, "  __ \\  \\  ||   | | \\   |'  :  | \\  \\ ,'|   |    ' |   |  |-, ");
    mvprintw(11, 27, " /  /`--'  /'   : |  ; .'|  |  '  '--'  '   : |.  \\'   :  ;/| ");
    mvprintw(12, 27, "'--'.     / |   | '`--'  |  :  :        |   | '_\\.'|   |    \\ ");
    mvprintw(13, 27, "  `--'---'  '   : |      |  | ,'        '   : |    |   :   .' ");
    mvprintw(14, 27, "            ;   |.'      `--''          ;   |,'    |   | ,'   ");
    mvprintw(15, 27, "            '---'                       '---'      `----'     ");

    attroff(COLOR_PAIR(5));
}
class listsnake {
    list<snake> snakes;
    list<snake>::iterator it;
public:
    listsnake() {

    }
    void createsnake(int x, int y) {
        snakes.push_front(snake(x, y));
    }
    void back() {
        snakes.pop_back();
    }
    void printsnake() {
        start_color();
        init_pair(1, COLOR_GREEN, COLOR_GREEN);
        init_pair(2, COLOR_RED, COLOR_RED);
        init_pair(3, COLOR_WHITE, COLOR_WHITE);
        for (it = snakes.begin(); it != snakes.end(); it++) {
            attron((COLOR_PAIR(1)));
            mvaddch((*it).getY(), (*it).getX(), 'O');
            attroff((COLOR_PAIR(1)));
        }
    }
    int getx() {
        snake logic = snakes.front();
        int x = logic.getX();
        return x;
    }
    int gety() {
        snake logic = snakes.front();
        int y = logic.getY();
        return y;
    }
    bool collision() {
        for (it = snakes.begin(); it != snakes.end(); it++) {
            if ((*it).getY() == gety() && (*it).getX() == getx() && it != snakes.begin())
                return true;
        }
        return false;
    }
};
class food {
    int fx, fy;
    int p = 0;
public:
    food()
    {}
    void createfood() {
        fx = (rand() % 76) + 22;
        fy = (rand() % 21) + 4;
        while ((fy == 8 && fx < 81 && fx > 40) || (fy == 21 && fx < 81 && fx > 40))
        {
            fx = (rand() % 76) + 22;
            fy = (rand() % 21) + 4;
        }
        while ( (fx == 50 && fy >= 12 && fy <= 16 ) || (fx == 70 && fy >= 12 && fy <= 16) )
        {
            fx = (rand() % 76) + 22;
            fy = (rand() % 21) + 4;
        }
    }
    int getfx() {
        return fx;
    }
    int getfy() {
        return fy; 
    }
    void print() {
        attron((COLOR_PAIR(2)));
        mvaddch(fy, fx, '@');
        attroff((COLOR_PAIR(2)));
    }
    void points(listsnake& e) {
        if (e.getx() == fx && e.gety() == fy) {
            beep();
            createfood();
            p++;
        }
        else
            e.back();
    }
    int points() {
        return p;
    }
};
class walls {
    int wy1,wy2, wx1,wx2;
public:
    walls()
    { }
    bool collisionwalls(listsnake& e) {

        //colisoes das barreiras horizontais
        if ( (e.gety() == 8 && e.getx() < 80 && e.getx() > 39) || (e.gety() == 21 && e.getx() < 80 && e.getx() > 39) )
        {
            return true;
        }
        //colisoes das barreiras verticais
        if ( (e.getx() == 50 && e.gety() >= 12 && e.gety() <= 16) || (e.getx() == 70 && e.gety() >= 12 && e.gety() <= 16) )
        {
            return true;
        }
        

    }
    void printwalls()
    {
        int wy2 = 12;
        int wx2 = 40;
        //walls horizontais
        for (int j = 1; j <= 40; j++)
        {
            attron((COLOR_PAIR(3)));
            mvprintw(8, wx2, "-");
            mvprintw(21, wx2, "-");
            attroff((COLOR_PAIR(3)));
            wx2++;
        }

        //walls verticais
        for (int k = 0; k < 5; k++)
        {
            attron((COLOR_PAIR(3)));
            mvprintw(wy2,50 , "|");
            mvprintw(wy2,70, "|");
            attroff((COLOR_PAIR(3)));
            wy2++;
        }
    }
};
class bord{
    int x1, x2, y1, y2;
public:
    bord() {
        x1 = 99;
        x2 = 21;
        y1 = 25;
        y2 = 4;

    }
    bool collision(listsnake& e) {
        if ((e.gety() > y1 || e.getx() >= x1 || e.gety() < y2 || e.getx() <= x2)) {
            return true;
        }
        else
            return false;
    }   
    void printbord() {
        int y2 = 4;
        int x2 = 21;
        //paredes horizontais
        for (int j = 1; j <= 79; j++)
        {
            attron((COLOR_PAIR(3)));
            mvprintw(3, x2, "-");

            mvprintw(26, x2, "-");
            attroff((COLOR_PAIR(3)));
            x2++;
        }

        //paredes verticais
        for (int k = 0; k < 22; k++)
        {
            attron((COLOR_PAIR(3)));
            mvprintw(y2, 21, "|");
            mvprintw(y2, 99, "|");
            attroff((COLOR_PAIR(3)));
            y2++;
        }

    }
};
int main() {

    // Init
    srand(time(NULL));
    initscr();
    start_color();
a://menu
    echo();
    curs_set(1);
    init_pair(1, COLOR_GREEN, COLOR_GREEN);//COR PREDEFINIDA DA COBRA COM ID = 1
    init_pair(2, COLOR_RED, COLOR_RED);//COR PREDEFINIDA DA COMIDA COM ID = 2
    init_pair(3, COLOR_WHITE, COLOR_WHITE);//COR DAS BARREIRAS COM ID = 3
    init_pair(4, COLOR_YELLOW, NULL);//COR DO SCORE COM ID = 4
    init_pair(6, COLOR_RED, NULL);

    print_snake();

    mvprintw(18, 40, "Insire your NickName(max. 10 characters): ");
    char* NICK = new char[11];
    getstr(NICK);

    if (strlen(NICK) >= 10)
    {
        erase();
        goto a;
    }
    else
    {
        erase();
        print_snake();
    }



    //caixa do menu
    WINDOW* win = newwin(10, 40, 17, 27);
    refresh();
    attron(COLOR_PAIR(5));
    box(win, 0, 0);
    attroff(COLOR_PAIR(5));
    mvwprintw(win, 0, 4, "MENU");
    mvwprintw(win, 2, 2, "1-EASY");
    mvwprintw(win, 3, 2, "2-MEDIUM");
    mvwprintw(win, 4, 2, "3-HARD");
    mvwprintw(win, 5, 2, "4-IMPOSSIBLE");
    mvwprintw(win, 7, 2, "5-EXIT");
    wrefresh(win);



   
    if (!has_colors())
    {
        printw("terminal doesnt support colours");
        getch();
        return 0;
    }


    int velocidade = 75;
    bool quit = false;
    noecho();
    curs_set(0);
    keypad(stdscr, TRUE);
    char option = getch();
    if (option == '1')
    {
        velocidade = 125;
        timeout(velocidade);
    }
    else if (option == '2')
    {
        timeout(velocidade);
    }
    else if (option == '3')
    {
        velocidade = 25;
        timeout(velocidade);
    }
    else if (option== '4')
    {
        velocidade = 1;
        timeout(velocidade);
        
    }
    else if (option == '5')
    {
        return 0;
    }
    else {
        timeout(75);
    }

   
    listsnake l;
    bord b;
    walls  wall;
    int p;
    int dir = 2;
    food c;
    c.createfood();
    int ch;

    for (int i = 0; i < 5; i++) // generate start snake
        l.createsnake(25 + i, 10);
    while (!quit) {
        // Input
        ch = getch();
        switch (ch) {
        case 'w':
        case KEY_UP:
            dir = 1;
            break;
        case 'd':
        case KEY_RIGHT:
            dir = 2;
            break;
        case 's':
        case KEY_DOWN:
            dir = 3;
            break;
        case 'a':
        case KEY_LEFT:
            dir = 4;
            break;
        case 'q':
            quit = true;
            break;
        }

        // Logic
        int xx = l.getx();
        int yy = l.gety();
        if (dir == 1) yy--; // move up
        else if (dir == 2) xx++; // move right
        else if (dir == 3) yy++; // move down
        else if (dir == 4) xx--; // move left

        l.createsnake(xx, yy);
        c.points(l);
        if (b.collision(l) == true) // collision with border
            quit = true;

        //output
        erase();
        c.print();
        l.printsnake();
        if (l.collision() == true)
            quit = true;
        if (option == '4') {
           
            wall.printwalls();
            if (wall.collisionwalls(l) == true)
                quit = true;
        }
        attron((COLOR_PAIR(4)));
        mvprintw(1, 40, "VOCE TEM %i PONTOS. DIGITE 'Q' PARA SAIR.\n", c.points());
        attroff((COLOR_PAIR(4)));
        b.printbord();
        refresh();
    }

    timeout(-1);
    erase();
    attron(COLOR_PAIR(6));
    mvprintw(2, 10, " ________  ________  _____ ______   _______           ________  ___      ___ _______   ________     ");
    mvprintw(3, 10, "|\\   ____\\|\\   __  \\|\\   _ \\  _   \\|\\  ___ \\         |\\   __  \\|\\  \\    /  /|\\  ___ \\ |\\   __  \\    ");
    mvprintw(4, 10, "\\ \\  \\___|\\ \\  \\|\\  \\ \\  \\\\\\__\\ \\  \\ \\   __/|        \\ \\  \\|\\  \\ \\  \\  /  / | \\   __/|\\ \\  \\|\\  \\   ");
    mvprintw(5, 10, " \\ \\  \\  __\\ \\   __  \\ \\  \\\\|__| \\  \\ \\  \\_|/__       \\ \\  \\\\\\  \\ \\  \\/  / / \\ \\  \\_|/_\\ \\   _  _\\  ");
    mvprintw(6, 10, "  \\ \\  \\|\\  \\ \\  \\ \\  \\ \\  \\    \\ \\  \\ \\  \\_|\\ \\       \\ \\  \\\\\\  \\ \\    / /   \\ \\ \\ \\_|\\ \\ \\  \\\\  \\| ");
    mvprintw(7, 10, "   \\ \\_______\\ \\__\\ \\__\\ \\__\\    \\ \\__\\ \\_______\\       \\ \\_______\\ \\__/ /     \\ \\_______\\ \\__\\\\ _\\ ");
    mvprintw(8, 10, "    \\|_______|\\|__|\\|__|\\|__|     \\|__|\\|_______|        \\|_______|\\|__|/       \\|_______|\\|__|\\|__|");



    mvprintw(13, 25, "YOUR SCORE: %i PONTOS", c.points());

    Score playerScore(c.points(), NICK);

    ifstream file("scores.txt");

    string str;

    list<Score> scores;
    list<Score>::iterator score_it;

    bool existe = false;

    if (!file.fail())
    {
        while (getline(file, str))
        {
            int pos = str.find(':');
            string name = str.substr(0, pos);
            string NICK2(NICK); // a classe string tem um construtor q trabalha com char* e ao utiliza "NICK" como argumento estamos a dizer que "NICK2" é igual a "NICK"
            stringstream scorestr(str.substr(pos + 1, str.length() - pos));
            int n = 0;
            scorestr >> n;

            //se os nomes for iguais o programa guarda o que tiver maior score
            if (NICK2 == name)
            {
                existe = true;
                if (c.points() > n)
                {
                    n = c.points();
                }

            }

            Score player(n, name);
            scores.push_back(player);
        }
    }
    //se o player não existir nos registos ele cria player e insere-o na lista
    if (!existe)
    {
        scores.push_back(playerScore);
    }

    file.close();
    //ordena os res de forma decrescente
    scores.sort(compare<Score>);
   
    

    ofstream f("scores.txt");
    int y = 14;

    for (int i = 0; !scores.empty() && i < 10; i++)
    {
        string s = "";
        for (int k = scores.front().get_name().length(); k < 10; k++)
        {
            s.push_back(' ');
        }
        f << scores.front().get_name() << ":" << scores.front().get_score() << "\n";

        mvprintw(y++, 75, "|%s%s  %9d|", scores.front().get_name().c_str(), s.c_str(), scores.front().get_score());

        scores.pop_front();

    }

    f.close();

    attron(A_UNDERLINE);
    mvprintw(12, 75, "                       ");
    mvprintw(13, 75, "|   HIGHEST SCORES    |");
    mvprintw(y, 75, "|=====================|");
    attroff(COLOR_PAIR(6));

    mvprintw(28, 90, "DIGITE 1 PARA VOLTAR AO MENU.");
    attroff(A_UNDERLINE);
    refresh();
    while (1)
    {
        char final = getch(); // espera pelo input
        switch (final)
        {
        case '1':
            erase();
            goto a;
        default:
            beep();
        }
    }


    endwin();

    return 0;
}