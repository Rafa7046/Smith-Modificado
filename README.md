# WEG Temperatura

## Estrutura do Projeto

- `main.py`: O arquivo principal em Python que utiliza a biblioteca em C.
- `scripts/`: Pasta contendo scripts necessários para a execução do projeto.
- `c_code/`: Pasta onde o código em C está localizado, incluido os códigos do controlador e da simulçao do sistema.
- `controllers/` e `utils/`: Pastas onde contém códigos para a chamada do código em C e para a visualização dos resultados.

Os principais arquivos são:
- `c_code/control_system.c`: Contém a função principal do controlador (`control`) e o loop principal de execução dos testes (`observer_controller_with_identification_rls`).
- `c_code/control_simulation.c`: Possui funções utilizadas pelo controlador e pela simulçao do sistema, como a função do identificador (`RLS`).
- `c_code/matrix_and_vector_ops.c`: Operações matriciais.
- `c_code/error_calculations.c`: Cálculo das métricas de erro para análise dos resultados do controlador.
- `c_code/real_system.c`: Funções para a simulação do sistema térmico.

## Requisitos

Certifique-se de ter o Python e o compilador C instalados em seu sistema. Além disso, você precisará de `setuptools` para compilar o código C.

## Instalação e Configuração

1. **Clone o repositório:**

    ```bash
    git clone https://github.com/Rafa7046/WEG_temperatura
    cd WEG_temperatura
    ```

2. Instale as dependências do Python:

    Crie um ambiente virtual e instale as bibliotecas necessárias:

    ```bash
    python -m venv venv
    source venv/bin/activate  # No Windows, use `venv\Scripts\activate`
    pip install -r requirements.txt
    ```

3. Compile o código C:

    Navegue até a pasta onde o código C está localizado e compile-o:

    ```bash
    .\\scripts\\build_c_code.sh  
    ```

4. Execute o arquivo principal:

    Navegue de volta à raiz do projeto e execute o arquivo principal `main.py`:

    ```bash
    python main.py
    ```

## Utilização

`main.py`: Este é o arquivo principal que você deve executar para rodar o projeto. Ele utiliza a biblioteca C para realizar operações específicas.
