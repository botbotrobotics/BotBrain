from langchain_openai import ChatOpenAI
from langchain_ollama import ChatOllama


def get_llm():
    llm = ChatOpenAI(model_name="gpt-4o", 
                    temperature=0)
    #llm = ChatOllama(
    #        #base_url="host.docker.internal:8080",
    #        model="qwq",
    #        temperature=0,
    #        num_ctx=8192,
    #   )
    return llm