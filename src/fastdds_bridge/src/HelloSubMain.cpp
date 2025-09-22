
// #include "HelloWorldPublisher.h"
#include "HelloWorldSubscriber.h"

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    HelloWorldSubscriber mysub;
    if (mysub.init())
    {
        mysub.run();
    }

    return 0;
}
