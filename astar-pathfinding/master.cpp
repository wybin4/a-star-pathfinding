#include <boost/graph/astar_search.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include<SFML/Graphics.hpp>

typedef boost::grid_graph<2> grid;
typedef boost::graph_traits<grid>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<grid>::vertices_size_type vertices_size_type;

//хэш-функция для вершин
struct vertex_hash
{
    size_t operator()(const vertex_descriptor& descriptor) const
    {
        size_t seed = 0;
        boost::hash_combine(seed, descriptor[0]); //объединяем хеш-значения х и у
        boost::hash_combine(seed, descriptor[1]);
        return seed;
    }
};

typedef boost::unordered_set<vertex_descriptor, vertex_hash> vertex_set; //структура данных для хранения вершин, только уникальные значения
//извлечение на основе хэш-функции
typedef boost::vertex_subset_complement_filter<grid, vertex_set>::type filtered_grid;//граф для фильтрации границ
//"удаляет" ненужные вершины, создавая новый граф
class field
{
private:
    grid underlying_grid; //вспомогательная сетка
    vertex_set barriers; //границы
    filtered_grid main_grid; //основная сетка
    vertex_set shortest_path; //вершины, являющиеся кратчайшем путём

    vertex_descriptor source; //точка старта
    vertex_descriptor goal; //целевая точка

    bool need_update; //при изменении точки начала/цели необходимо обновить картинку и пересчитать путь
    //создаём двумерную вспомогательную сетку указанной размерности
    grid create_grid(size_t x, size_t y)
    {
        boost::array<size_t, 2> lengths = { {x, y} };
        return grid(lengths);
    }
    bool has_barrier(vertex_descriptor vertex) const //если попали на границу, то возвращаем true
    {
        return barriers.find(vertex) != barriers.end();
    }
    //фильтруем вспомогательную сетку и получаем основную
    filtered_grid create_barrier_grid()
    {
        return boost::make_vertex_subset_complement_filter(underlying_grid, barriers);
    }

public:
    //изначально need_update = true тк уже нужно увидеть решение
    field() : underlying_grid(create_grid(0, 0)), main_grid(create_barrier_grid()) { need_update = true;  source = vertex(0, underlying_grid), goal = vertex(num_vertices(underlying_grid) - 1, underlying_grid); };
    field(size_t x, size_t y) : underlying_grid(create_grid(x, y)), main_grid(create_barrier_grid()) { need_update = true;  source = vertex(0, underlying_grid), goal = vertex(num_vertices(underlying_grid) - 1, underlying_grid); };

    vertex_descriptor get_source() const { return source; }
    vertex_descriptor get_goal() const { return goal; }

    void set_source(const vertex_descriptor& src) { source = src; }
    void set_goal(const vertex_descriptor& g) { goal = g; }

    bool solve(); //ключевая функция поиска пути
    bool solved() const { return !shortest_path.empty(); }
    bool solution_contains(vertex_descriptor vertex) const
    {
        return shortest_path.find(vertex) != shortest_path.end();
    }
    void update(); //обновляем картинку
    void draw(); //создаём картинку
    friend void random_barriers(field&);
};

//эвклидова метрика для расстояния между текущей вершиной и целью
class euclidean_heuristic :public boost::astar_heuristic<filtered_grid, double>
{
private:
    vertex_descriptor goal;
public:
    euclidean_heuristic(vertex_descriptor g) :goal(g) {};
    double operator()(vertex_descriptor vertex)
    {
        double x = goal[0] - vertex[0];
        double y = goal[1] - vertex[1];
        return sqrt(x * x + y * y);
    }
};

//возбуждаем исключение, если нашли путь
struct found_goal {};

//класс-посетитель, чей метод examine_vertex возбуждает исключение
struct astar_goal_visitor : public boost::default_astar_visitor
{
private:
    vertex_descriptor goal;
public:
    astar_goal_visitor(vertex_descriptor g) : goal(g) {};
    void examine_vertex(vertex_descriptor vertex, const filtered_grid&)
    {
        if (vertex == goal)
            throw found_goal();
    }
};

//обходим граф-сетку. возвращаем true, если нашли путь
bool field::solve()
{
    typedef boost::unordered_map<vertex_descriptor, double, vertex_hash> dist_map;
    typedef boost::unordered_map<vertex_descriptor, vertex_descriptor, vertex_hash> pred_map;

    boost::static_property_map<double> weight(1); //расстояние между клетками сетки
    pred_map predecessor; //предшественники
    boost::associative_property_map<pred_map> pred_pmap(predecessor);
    dist_map distance; //карта фактических расстояний
    boost::associative_property_map<dist_map> dist_pmap(distance);

    vertex_descriptor src = get_source();
    vertex_descriptor g = get_goal();
    euclidean_heuristic heuristic(g);
    astar_goal_visitor visitor(g);
    shortest_path.clear(); //обязательное очищение, чтобы пути не накладывались друг на друга
    try
    {//поиск пути с функцией, которая принимает ссылку на граф-сетку, координаты исходной точки, метрику
        //из predecessor извлекается результат и расстояние distance
        astar_search(main_grid, src, heuristic,
            boost::weight_map(weight)
            .predecessor_map(pred_pmap)
            .distance_map(dist_pmap)
            .visitor(visitor));
    }
    catch (found_goal fg)
    {
        //идём в обратном направлении от цели через цепочку предшественников, добавляя
        //вершины к пути решения
        for (vertex_descriptor vertex = g; vertex != src; vertex = predecessor[vertex])
            shortest_path.insert(vertex);
        shortest_path.insert(src);
        return true;
    }
    return false;
}

//размеры окна
constexpr float window_width = 900;
constexpr float window_height = 450;

sf::RenderWindow window(sf::VideoMode(window_width, window_height), "A* Pathfinder", sf::Style::Close);
sf::Event evnt;

const sf::Vector2f tile_size = { 45, 45 }; //размер клетки
const size_t collumns_x = window_width / tile_size.x;
const size_t collumns_y = window_height / tile_size.y;

//цвета фигурок
const sf::Color outer_border_color = sf::Color::Black;
const sf::Color source_color = sf::Color::Blue;
const sf::Color goal_color = sf::Color::Red;
const sf::Color path_color = sf::Color::Green;
const sf::Color barrier_color = sf::Color::Black;

void field::update()
{
    //координаты мышки
    int x = sf::Mouse::getPosition(window).x / tile_size.x;
    int y = sf::Mouse::getPosition(window).y / tile_size.y;

    if (x >= 0 && x < collumns_x && y >= 0 && y < collumns_y)
    {
        vertex_descriptor coordinates = { { x, vertices_size_type(y) } }; //преобразуем в нужный тип для сравнения
        if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left))
        {//меняем начальное положение по левой кнопке мыши
            if (get_goal() != coordinates and !has_barrier(coordinates)) //чтобы не совпали положения начала и цели и мы не попали в границу
            {
                set_source(coordinates);
                need_update = true; //нужно обновить картинку
            }
        }
        if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Right))
        {//меняем положение цели по левой кнопке мыши
            if (get_source() != coordinates and !has_barrier(coordinates))
            {
                set_goal(coordinates);
                need_update = true;
            }
        }
    }
    if (need_update) //если нужно обновить картинку
    {
        solve(); //заново обходим граф
        need_update = false; //уже не нужно
    }
}

void field::draw()
{
    for (size_t x = 0; x < collumns_x; x++)
    {
        for (size_t y = 0; y < collumns_y; y++)
        {
            vertex_descriptor coordinates = { {x, vertices_size_type(y)} }; //правильный формат координат

            sf::RectangleShape first; //клетки 
            sf::RectangleShape second;
            first.setSize(tile_size); //размер
            first.setPosition(x * tile_size.x, y * tile_size.y); //местоположение по координатам поля
            first.setOutlineThickness(tile_size.x / 30); //толщина
            first.setOutlineColor(outer_border_color); //цвет внешних границ

            if (coordinates == get_goal()) //если это цель, то красный цвет
                first.setFillColor(goal_color);
            else if (coordinates == get_source()) //если начало, то синий
                first.setFillColor(source_color);
            else if (has_barrier(coordinates)) //если в клетке есть барьер, то чёрный
                first.setFillColor(barrier_color);

            second.setOutlineThickness(0); //для пути нулевая толщина границ
            if (solution_contains(coordinates) and coordinates != get_source() and coordinates != get_goal())
            { //если клетка не содержит начала/конца пути, то
                second.setFillColor(path_color); //зелёный цвет
                second.setSize({ tile_size.x / 2, tile_size.y / 2 }); //и размер поменьше
                second.setPosition(x * tile_size.x + tile_size.x / 4, y * tile_size.y + tile_size.y / 4); //соответствующее расположение
            }
            //отрисовываем
            window.draw(first);
            window.draw(second);
        }
    }
}

void random_barriers(field& f) //случайное размещение барьеров
{
    vertices_size_type n = num_vertices(f.underlying_grid);
    vertex_descriptor src = f.get_source();
    vertex_descriptor g = f.get_goal();
    int barriers = n / 4; //занимают четверть пространства
    while (barriers > 0)
    {
        size_t direction = rand() % 2; //горизонтальное или вертикальное
        vertices_size_type wall = rand() % 4;
        vertex_descriptor vert = vertex(rand() % (n - 1), f.underlying_grid);
        while (wall)
        {
            //в начальной и конечной точке не может быть барьеров
            if (vert != src and vert != g)
            {
                wall--;
                if (!f.has_barrier(vert)) //если нет барьера
                {
                    f.barriers.insert(vert); //то будет
                    barriers--; //уменьшаем общее число
                }
            }
            vertex_descriptor next_vert = f.underlying_grid.next(vert, direction); //следующая вершина в этом направлении
            if (vert == next_vert)//останавливаем создание стены при достижении края поля            
                break;
            vert = next_vert;
        }
    }
}

int main()
{
    //параметры поля, оно задано как 20 на 10

    srand(time(NULL));
    field f(collumns_x, collumns_y); //создаём поле
    random_barriers(f); //формируем барьеры
    window.setFramerateLimit(60);

    while (window.isOpen())
    {
        while (window.pollEvent(evnt)) //закрытие окна
            if (evnt.type == sf::Event::Closed) window.close();
        f.update();
        window.clear(sf::Color::White);
        f.draw();
        window.display();
    }
    return 0;
}

