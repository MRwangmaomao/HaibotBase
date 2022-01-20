
#ifndef LIB_H
#define LIB_H

//define uma estrutura No que contém um valor e um ponteiro para um próximo nó.
//uso geral em fila e pilha.
typedef struct No {
	int valor;
	struct No *prox;
} No;

//define uma estrutura Fila que contém um nó inicial e um indicador da sua quantidade de elementos.
typedef struct {
	No *inicio;
	int tamanho;
} Fila;

Fila* criar_fila(); //aloca uma nova fila.
void inserir_final(Fila *fl, int valor); //insere um valor no final da fila.
int remover_inicio(Fila *fl); //remove um valor da fila, de acordo com FIFO.
No* ultimo_no(); //retorna o ultimo ponteiro da fila.
int fila_vazia(Fila *fl); //retorna 1 (true) ou 0 (false), indicando se a fila está vazia.
void liberar_fila(Fila *fl); //esvazia a fila e libera a memória alocada.
void imprimir_fila(Fila *fl); //imprime a fila.

//define uma estrutura com os atributos contidos em uma imagem de formato PGM.
typedef struct {
	char chave[2];
	unsigned short l, c;
	unsigned char max, *mtr;
} PGM;

P//GM ler_imagem(FILE *arq, PGM img); //decodifica uma imagem PGM.
//void ler_matriz(FILE *arq, PGM img); //lê a matriz de uma imagem PGM, de acordo com seu tipo.
//void ler_mtr_p2(FILE *arq, unsigned char *mtr, unsigned short l, unsigned short c); //lê matriz P2 (ascii).
//void ler_mtr_p5(FILE *arq, unsigned char *mtr, unsigned short l, unsigned short c); //lê matriz P5 (bin).
void imprimir_mtr(unsigned char *mtr, unsigned short l, unsigned short c); //imprime a matriz de uma imagem PGM.

unsigned char* wshed(unsigned char *mtr, unsigned short l, unsigned short c); //aplica o algoritmo watershed à matriz da imagem.
#endif
